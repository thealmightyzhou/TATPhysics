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

	static bool CalcTimeOfImpact
	(
		TATVector3 pt, const TATVector3& vp,
		TATVector3 face[3], TATVector3 vf[3],
		float& t, int iterateNum, float margin
	)
	{
		TATVector3 originPos = pt;

		int iteNum = 0;
		TATVector3 normal = ((face[1] - face[0]).Cross(face[2] - face[0])).SafeNormalize();
		float dist = (pt - face[0]).Dot(normal) - margin;

		if (dist < 0)
			return false;

		float vel[3];
		vel[0] = vf[0].Dot(normal);
		vel[1] = vf[1].Dot(normal);
		vel[2] = vf[2].Dot(normal);

		float rel_vel = vel[_MaxOfArray<float>(vel, 3)] - vp.Dot(normal);

		float time = dist / rel_vel;
		if (1.0 < time || time < 0.0)
			return false;

		float dt = time;
		while (iteNum < iterateNum)
		{
			pt += vp * dt;
			face[0] += vf[0] * dt;
			face[1] += vf[1] * dt;
			face[2] += vf[2] * dt;

			float w[3];
			float dis = TATGeometryUtil::ClosetPtOnTri(pt, face[0], face[1], face[2], w).Distance(pt);
			if (dis < (margin + COLLIDE_EPS) && TATVoronoiUtil::PtInFaceVor(pt, face[0], face[1], face[2]))
			{
				t = time;
				return true;
			}

			normal = ((face[1] - face[0]).Cross(face[2] - face[0])).SafeNormalize();
			vel[0] = vf[0].Dot(normal);
			vel[1] = vf[1].Dot(normal);
			vel[2] = vf[2].Dot(normal);

			dist = (pt - face[0]).Dot(normal) - margin;

			if (dist < -TAT_EPSILON2)
				return false;

			rel_vel = vel[_MaxOfArray<float>(vel, 3)] - vp.Dot(normal);

			dt = dist / rel_vel;
			time += dt;

			if (1.0 + TAT_EPSILON < time || time < -TAT_EPSILON)
				return false;

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

	static bool CalcTimeOfImpact
	(
		TATVector3* face0,
		TATVector3* vel0,
		TATVector3* face1,
		TATVector3* vel1,
		int ite_num,
		float margin,
		float collide_radius,
		float& t
	)
	{
		t = 0;

		TATVector3 inte_face0[3]{ face0[0],face0[1],face0[2] };
		TATVector3 inte_face1[3]{ face1[0],face1[1],face1[2] };

		float distance;
		if (!TATSATDistSolver::TriangleDistance(inte_face0, inte_face1, distance))
			return false;

		if (distance < margin + collide_radius)
			return true;

		TATVector3 edge0[3]{ inte_face0[1] - inte_face0[0],inte_face0[2] - inte_face0[1],inte_face0[2] - inte_face0[0] };
		TATVector3 edge1[3]{ inte_face1[1] - inte_face1[0],inte_face1[2] - inte_face1[1],inte_face1[2] - inte_face1[0] };
		TATVector3 normal0 = ((inte_face0[1] - inte_face0[0]).Cross(inte_face0[2] - inte_face0[0])).Normalized();
		TATVector3 normal1 = ((inte_face1[1] - inte_face1[0]).Cross(inte_face1[2] - inte_face1[0])).Normalized();

		TATVector3 ct0 = (inte_face0[0] + inte_face0[1] + inte_face0[2]) / 3;
		TATVector3 ct1 = (inte_face1[0] + inte_face1[1] + inte_face1[2]) / 3;

		TATVector3 correct_dir = ct0 - ct1;

		TATVector3 test_dirs[11]
		{
			edge0[0].Cross(edge1[0]).Normalized(),
			edge0[0].Cross(edge1[1]).Normalized(),
			edge0[0].Cross(edge1[2]).Normalized(),
			edge0[1].Cross(edge1[0]).Normalized(),
			edge0[1].Cross(edge1[1]).Normalized(),
			edge0[1].Cross(edge1[2]).Normalized(),
			edge0[2].Cross(edge1[0]).Normalized(),
			edge0[2].Cross(edge1[1]).Normalized(),
			edge0[2].Cross(edge1[2]).Normalized(),
			normal0,
			normal1
		};

		float nvel0[3];
		float nvel1[3];
		float minTime = TAT_MAX;
		float time;
		for (int i = 0; i < 11; ++i)
		{
			if (test_dirs[i].Dot(correct_dir) < 0)
				test_dirs[i] = -test_dirs[i];

			float dist;
			if (!Support(inte_face0, inte_face1, test_dirs[i], dist))
				continue;

			for (int j = 0; j < 3; ++j)
			{
				nvel0[j] = vel0[j].Dot(test_dirs[i]);
				nvel1[j] = vel1[j].Dot(test_dirs[i]);
			}

			float rel_vel = nvel1[_MaxOfArray<float>(nvel1, 3)] - nvel0[_MinOfArray<float>(nvel0, 3)];
			time = (dist - margin) / rel_vel;

			if (time < 0 || 1 < time)
				continue;
			else
				if (time < minTime)
					minTime = time;
		}


		int ite = 0;
		while (ite < ite_num)
		{
			t += minTime;
			minTime = TAT_MAX;

			for (int i = 0; i < 3; ++i)
			{
				inte_face0[i] += t * vel0[i];
				inte_face1[i] += t * vel1[i];
			}
			if (!TATSATDistSolver::TriangleDistance(inte_face0, inte_face1, distance))
				return false;

			if (distance < margin + collide_radius)
				return true;

			edge0[0] = inte_face0[1] - inte_face0[0];
			edge0[1] = inte_face0[2] - inte_face0[1];
			edge0[2] = inte_face0[2] - inte_face0[0];
			edge1[0] = inte_face1[1] - inte_face1[0];
			edge1[1] = inte_face1[2] - inte_face1[1];
			edge1[2] = inte_face1[2] - inte_face1[0];
			normal0 = ((inte_face0[1] - inte_face0[0]).Cross(inte_face0[2] - inte_face0[0])).Normalized();
			normal1 = ((inte_face1[1] - inte_face1[0]).Cross(inte_face1[2] - inte_face1[0])).Normalized();
			ct0 = (inte_face0[0] + inte_face0[1] + inte_face0[2]) / 3;
			ct1 = (inte_face1[0] + inte_face1[1] + inte_face1[2]) / 3;
			correct_dir = ct0 - ct1;

			test_dirs[0] = edge0[0].Cross(edge1[0]).Normalized();
			test_dirs[1] = edge0[0].Cross(edge1[1]).Normalized();
			test_dirs[2] = edge0[0].Cross(edge1[2]).Normalized();
			test_dirs[3] = edge0[1].Cross(edge1[0]).Normalized();
			test_dirs[4] = edge0[1].Cross(edge1[1]).Normalized();
			test_dirs[5] = edge0[1].Cross(edge1[2]).Normalized();
			test_dirs[6] = edge0[2].Cross(edge1[0]).Normalized();
			test_dirs[7] = edge0[2].Cross(edge1[1]).Normalized();
			test_dirs[8] = edge0[2].Cross(edge1[2]).Normalized();
			test_dirs[9] = normal0;
			test_dirs[10] = normal1;
			
			for (int i = 0; i < 11; ++i)
			{
				if (test_dirs[i].Dot(correct_dir) < 0)
					test_dirs[i] = -test_dirs[i];

				float dist;
				if (!Support(inte_face0, inte_face1, test_dirs[i], dist))
					continue;

				for (int j = 0; j < 3; ++j)
				{
					nvel0[j] = vel0[j].Dot(test_dirs[i]);
					nvel1[j] = vel1[j].Dot(test_dirs[i]);
				}

				float rel_vel = nvel1[_MaxOfArray<float>(nvel1, 3)] - nvel0[_MinOfArray<float>(nvel0, 3)];
				time = (dist - margin) / rel_vel;

				if (time < 0 || 1 < time)
					continue;
				else
					if (time < minTime)
						minTime = time;
			}

			ite++;
		}

	}

	static bool Support(TATVector3* face0, TATVector3* face1, const TATVector3& dir, float& dis)
	{
		float proj0[3]{ face0[0].Dot(dir), face0[1].Dot(dir), face0[2].Dot(dir) };
		float proj1[3]{ face1[0].Dot(dir), face1[1].Dot(dir), face1[2].Dot(dir) };
		TATRange range0(proj0[_MinOfArray<float>(proj0, 3)], proj0[_MaxOfArray<float>(proj0, 3)]);
		TATRange range1(proj1[_MinOfArray<float>(proj1, 3)], proj1[_MaxOfArray<float>(proj1, 3)]);
		if (range0.Coincide(range1))
			return false;
		
		float dist[2]{ range0.m_MaxRange - range1.m_MinRange,range1.m_MaxRange - range0.m_MinRange };
		dis = -dist[_MinOfArray<float>(dist, 2)];
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

	static TATMatrix3 ImpulseMatrix(float ima, const TATMatrix3& iia, const TATVector3& ra,
									float imb, const TATMatrix3& iib, const TATVector3& rb)
	{
		(MassMatrix(ima, iia, ra) + MassMatrix(imb, iib, rb)).Inverse();
	}
};