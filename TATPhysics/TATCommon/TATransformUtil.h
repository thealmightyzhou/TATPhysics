#pragma once
#include "TATransform.h"

struct TATRigidBodyData;
class TATRigidBody;

#define TAT_ANGULAR_MOTION_THRESHOLD float(0.5) * TAT_HALF_PI

class TATransformUtil
{
public:
	static void IntegrateTransform(const TATransform& curTr, const TATVector3& linVel, const TATVector3& angVel, float timeStep, TATransform& predictTr)
	{
		predictTr.SetOrigin(curTr.GetOrigin() + linVel * timeStep);

		TATVector3 axis;
		float fAngle = angVel.Length();
		//limit the angular motion
		if (fAngle * timeStep > TAT_ANGULAR_MOTION_THRESHOLD)
		{
			fAngle = TAT_ANGULAR_MOTION_THRESHOLD / timeStep;
		}

		if (fAngle < float(0.001))
		{
			// use Taylor's expansions of sync function
			axis = angVel * (float(0.5) * timeStep - (timeStep * timeStep * timeStep) * (float(0.020833333333)) * fAngle * fAngle);
		}
		else
		{
			// sync(fAngle) = sin(c*fAngle)/t
			axis = angVel * (sin(float(0.5) * fAngle * timeStep) / fAngle);
		}
		TATQuaternion dorn(axis.X, axis.Y, axis.Z, cos(fAngle * timeStep * float(0.5)));
		TATQuaternion orn0 = curTr.GetRotation();

		TATQuaternion predictedOrn = dorn * orn0;
		predictedOrn.Normalize();

		predictTr.SetRotation(predictedOrn);
	}

	static void IntegrateTransform(TATRigidBodyData* body, float timeStep, float angularDamping);

	static void IntegrateTransform(TATRigidBody* body, float timeStep, float angularDamping);

	static void CalculateVelocity(const TATransform& tr0, const TATransform& tr1, float timeStep, TATVector3& linVel, TATVector3& angVel)
	{
		linVel = (tr1.GetOrigin() - tr0.GetOrigin()) / timeStep;
		TATVector3 axis;
		float angle;
		CalculateDiffAxisAngle(tr0, tr1, axis, angle);
		angVel = axis * angle / timeStep;
	}

	static void CalculateDiffAxisAngle(const TATransform& tr0, const TATransform& tr1, TATVector3& axis, float& angle)
	{
		TATMatrix3 deltaMat = tr1.GetBasis() * tr0.GetBasis().Inverse();
		TATQuaternion dorn;
		deltaMat.GetRotation(dorn);

		dorn.Normalize();
		angle = dorn.GetAngle();
		axis = TATVector3(dorn.X, dorn.Y, dorn.Z);
		float len2 = axis.Length2();
		if (len2 < TAT_EPSILON * TAT_EPSILON)
		{
			axis = TATVector3::UnitX();
		}
		else
		{
			axis /= sqrt(len2);
		}
	}
};