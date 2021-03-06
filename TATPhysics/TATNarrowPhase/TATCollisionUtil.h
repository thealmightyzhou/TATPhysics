#pragma once
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATCore.h"
#include "../TATGeometry/TATGeometryComputer.h"
#include "TATSAT.h"
#include "../TATCommon/TATransformUtil.h"
#include "../TATDynamics/TATRigidBody.h"

#define MAX_ITERATIONS 64
#define COLLIDE_EPS 0.04f

struct TATCCDResult
{
public:

	TATVector3 m_CollidePt;
	TATVector3 m_CollideNormal;
	float m_Fraction;
	float m_AllowedPenetration;
};


class TATCollisionUtil
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

		TATSATDistPack cd;
		float dist = TATSATDistSolver::SolveConvexDistance(csA, csB, fromA, fromB, cd) + allowedPenetration;
		n = cd.m_Normal;
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

			TATSATDistPack data;

			dist = allowedPenetration + TATSATDistSolver::SolveConvexDistance(csA, csB, interpolatedTrA, interpolatedTrB, data);
			cp = data.m_ClostPtA;
			n = data.m_Normal;

			numIter++;
			if (numIter > MAX_ITERATIONS)
				return false;
		}

		res.m_CollideNormal = n;
		res.m_CollidePt = cp;
		res.m_Fraction = lambda;

		return true;
	}

	static bool GetPtTriTOIWithRadius
	(
		TATVector3 pt, const TATVector3& vp,
		TATVector3 face[3], TATVector3 vf[3],
		float& t, int iterateNum, float margin,
		float collide_radius, TATVector3& normal
	)
	{
		TATVector3 p = pt;
		TATVector3 f[3]{ face[0],face[1],face[2] };
		int iteNum = 0;
		TATVector3 norm;
		TATVector3 ptOnTri;
		float dist;
		float vn[3];
		float rel_vel;

		float dt;
		while (iteNum < iterateNum)
		{
			float w[3];
			dist = TATGeometryUtil::ClosetPtOnTri(p, f[0], f[1], f[2], w).Distance(p) - margin;
			if (dist < collide_radius)
			{
				normal = p - (f[0] * w[0] + f[1] * w[1] + f[2] * w[2]);
				return true;
			}

			ptOnTri = f[0] * w[0] + f[1] * w[1] + f[2] * w[2];
			norm = p - ptOnTri;
			vn[0] = vf[0].Dot(normal);
			vn[1] = vf[1].Dot(normal);
			vn[2] = vf[2].Dot(normal);

			rel_vel = vn[_MaxOfArray<float>(vn, 3)] - vp.Dot(normal);

			if (rel_vel <= 0)
				return false;

			dt = dist / rel_vel;
			t += dt;
			if (t > 1)
				return false;

			p = pt + vp * t;
			f[0] = face[0] + vf[0] * t;
			f[1] = face[1] + vf[1] * t;
			f[2] = face[2] + vf[2] * t;

			iteNum++;
		}

		return false;
	}

	//return particle collide rigid face TOI @t[0,1]
	static bool CalcTimeOfImpact
	(
		const TATVector3& pt, 
		const TATVector3& vel,
		TATRigidBody* rigid,
		const TATVector3& linVel,
		const TATVector3& angVel,
		int faceindex,
		float dt,
		float margin,
		float collideRadius,
		int iterate,
		float& t
	)
	{

		const TATransform& tr = rigid->GetWorldTransform();
		TATransform predictTr;
		TATransformUtil::IntegrateTransform(tr, linVel, angVel, dt, predictTr);

		TATCollideShapeConvex* convex = rigid->m_CollideShape->Cast<TATCollideShapeConvex>();

		const TATPhyFace& face = convex->m_CollideMeshData.m_Faces[faceindex];
		TATVector3 vertices[3]{ tr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[0]].m_Position ,
								tr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[1]].m_Position ,
								tr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[2]].m_Position };

		TATVector3 predicts[3]{ predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[0]].m_Position ,
								predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[1]].m_Position ,
								predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[2]].m_Position };

		const TATVector3 velocity[3]{
			predicts[0] - vertices[0],
			predicts[1] - vertices[1],
			predicts[2] - vertices[2]};

		TATVector3 normal = (vertices[1] - vertices[0]).Cross(vertices[2] - vertices[0]).Normalized();

		float dist = (pt - vertices[0]).Dot(normal) - margin;
		if (dist < -TAT_EPSILON2)
			return false;

		float normVel[3]{ velocity[0].Dot(normal),
						  velocity[1].Dot(normal),
						  velocity[2].Dot(normal) };

		float max_vel = normVel[_MaxOfArray<float>(normVel, 3)];
		float rel_vel = max_vel - vel.Dot(normal);

		float timeRate = dist / rel_vel;
		if (1.0f + TAT_EPSILON < timeRate || timeRate < -TAT_EPSILON)
			return false;

		int ite = 0;
		float elapsedTime = timeRate * dt;

		TATVector3 interplotePt;
		float deltaTime = 0;
		while (ite < iterate)
		{
			interplotePt = vel * timeRate + pt;
			TATransformUtil::IntegrateTransform(tr, linVel, angVel, elapsedTime, predictTr);
			vertices[0] = predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[0]].m_Position;
			vertices[1] = predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[1]].m_Position;
			vertices[2] = predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[2]].m_Position;
			normal = (vertices[1] - vertices[0]).Cross(vertices[2] - vertices[0]).Normalized();

			dist = (interplotePt - vertices[0]).Dot(normal) - margin;
			if (dist < -TAT_EPSILON)
				return false;
			else if (0 < dist && dist < collideRadius)
			{
				t = timeRate;
				return true;
			}

			normVel[0] = velocity[0].Dot(normal);
			normVel[1] = velocity[1].Dot(normal);
			normVel[2] = velocity[2].Dot(normal);

			max_vel = normVel[_MaxOfArray<float>(normVel, 3)];
			rel_vel = max_vel - vel.Dot(normal);

			deltaTime = dist / rel_vel;
			if (1.0f + TAT_EPSILON < deltaTime || deltaTime < -TAT_EPSILON)
				return false;
			timeRate += deltaTime;
			if (1.0f + TAT_EPSILON < timeRate || timeRate < -TAT_EPSILON)
				return false;

			elapsedTime = timeRate * dt;

			ite++;
		}

		return false;
	}

	static bool GetTriTriTOIWithRadius
	(
		TATVector3* face0,
		TATVector3* vel0,
		TATVector3* face1,
		TATVector3* vel1,
		int ite_num,
		float margin,
		float collide_radius,
		float& t,
		TATVector3& normal,
		float w1[3],
		float w2[3]
	)
	{
		t = 0;
		int ite = 0;
		TATVector3 tri0[3]{ face0[0],face0[1],face0[2] };
		TATVector3 tri1[3]{ face1[0],face1[1],face1[2] };

		float nvel0[3];
		float nvel1[3];

		TATVector3 norm;
		float dt;
		float dist;
		TATSATDistPack dp;
		
		while (ite < ite_num)
		{
			dist = TATSATDistSolver::SolveTriangleDistance(tri0, tri1, dp) - margin;
			if (dist < collide_radius)
			{
				normal = (dp.m_ClostPtB - dp.m_ClostPtA).Normalized();
				TATGeometryUtil::ClosetPtOnTri(dp.m_ClostPtA, tri0[0], tri0[1], tri0[2], w1);
				TATGeometryUtil::ClosetPtOnTri(dp.m_ClostPtB, tri1[0], tri1[1], tri1[2], w2);
				return true;
			}

			norm = (dp.m_ClostPtB - dp.m_ClostPtA).Normalized();
			for (int i = 0; i < 3; ++i)
			{
				nvel0[i] = vel0[i].Dot(norm);
				nvel1[i] = vel1[i].Dot(norm);
			}

			float rel_vel = nvel0[_MaxOfArray<float>(nvel0, 3)] - nvel1[_MinOfArray<float>(nvel1, 3)];
			if (rel_vel <= 0)
			{
				return false;
			}
			dt = dist / rel_vel;
			t += dt;

			if (t > 1)
				return false;

			for (int i = 0; i < 3; ++i)
			{
				tri0[i] = face0[i] + t * vel0[i];
				tri1[i] = face1[i] + t * vel1[i];
			}

			ite++;
		}

	}

	static TATMatrix3 MassMatrix(float im, const TATMatrix3& iwi, const TATVector3& r)
	{
		TATMatrix3 rx;
		r.GetSkewSymmetricMatrix(&rx.Ele[0], &rx.Ele[1], &rx.Ele[2]);
		return (TATMatrix3::Diagonal(im) - rx * iwi * rx);
	}

	static TATMatrix3 ImpulseMatrix(float dt, float ima, float imb, const TATMatrix3& iwi, const TATVector3& r)
	{
		return (TATMatrix3::Diagonal(1.0f / dt) * (TATMatrix3::Diagonal(ima) + MassMatrix(imb, iwi, r)).Inverse());
	}

	static TATMatrix3 ImpulseMatrix(float ima, float imb, const TATMatrix3& iwi, const TATVector3& r)
	{
		return (TATMatrix3::Diagonal(ima) + MassMatrix(imb, iwi, r)).Inverse();
	}

	static TATMatrix3 ImpulseMatrix(float ima, const TATMatrix3& iia, const TATVector3& ra,
									float imb, const TATMatrix3& iib, const TATVector3& rb)
	{
		(MassMatrix(ima, iia, ra) + MassMatrix(imb, iib, rb)).Inverse();
	}
};