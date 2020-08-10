#pragma once
#include "TATSAT.h"
#include "../TATCommon/TATransformUtil.h"

#define MAX_ITERATIONS 64

struct TATCCDResult
{
public:

	TATVector3 m_CollidePt;
	TATVector3 m_CollideNormal;
	float m_Fraction;
	float m_AllowedPenetration;
};

class TATCCD
{
public:
	static bool CalcTimeOfImpact(TATCollideShapeConvex* csA, const TATransform& fromA, const TATransform& toA,
								 TATCollideShapeConvex* csB, const TATransform& fromB, const TATransform& toB,
								 TATCCDResult& res)
	{

		float allowedPenetration = res.m_AllowedPenetration;

		TATVector3 linVelA, linVelB, angVelA, angVelB;
		TATransformUtil::CalculateVelocity(fromA, toA, float(1.0f), linVelA, angVelA);
		TATransformUtil::CalculateVelocity(fromB, toB, float(1.0f), linVelB, angVelB);

		float boundingRadiusA = csA->GetAngularMotionDisc();
		float boundingRadiusB = csB ? csB->GetAngularMotionDisc() : 0;

		float maxAngularProjectedVelocity = angVelA.Length() * boundingRadiusA + angVelB.Length() * boundingRadiusB;
		TATVector3 relLinVel = linVelA - linVelB; //
		float relLinVelLength = relLinVel.Length();

		if (relLinVelLength + maxAngularProjectedVelocity == 0.0f)
			return false;

		float lambda = 0.0f;
		float lastLambda = lambda;

		TATVector3 cp;
		TATVector3 n(0, 0, 0);
		bool hasRes = false;

		int numIter = 0;

		float radius = 0.001f;

		TATSATCollideData cd;
		float dist = TATSATDist::ConvexDistance(csA, csB, fromA, fromB, cd) + allowedPenetration;
		n = cd.m_CollideNormal; //B2A
		float projectedLinearVelocity = relLinVel.Dot(n);

		if (projectedLinearVelocity + maxAngularProjectedVelocity <= TAT_EPSILON)
			return false;

		while (dist > radius)
		{
			float dlambda = 0.0f;

			projectedLinearVelocity = relLinVel.Dot(n);

			if (projectedLinearVelocity + maxAngularProjectedVelocity <= TAT_EPSILON)
				return false;

			dlambda = dist / (projectedLinearVelocity + maxAngularProjectedVelocity);

			lambda += dlambda;

			if (lambda < float(0.0f) || float(1.0f) < lambda)
				return false;

			if (lambda <= lastLambda)
				return false;

			lastLambda = lambda;

			TATransform interpolatedTrA, interpolatedTrB, relativeTr;

			TATransformUtil::IntegrateTransform(fromA, linVelA, angVelA, lambda, interpolatedTrA);
			TATransformUtil::IntegrateTransform(fromB, linVelB, angVelB, lambda, interpolatedTrB);
			relativeTr = interpolatedTrB.InverseTimes(interpolatedTrA);

			TATSATCollideData data;

			dist = allowedPenetration + TATSATDist::ConvexDistance(csA, csB, interpolatedTrA, interpolatedTrB, data);
			cp = data.m_CollidePtA;
			n = data.m_CollideNormal;

			numIter++;
			if (numIter > MAX_ITERATIONS)
				return false;
		}

		res.m_CollideNormal = n;
		res.m_CollidePt = cp;
		res.m_Fraction = lambda;

		return true;
	}
};