#pragma once
#include "TATSoftRigidCollisionData.h"
#include "../TATBroadPhase/TATBvh.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"
#include "../TATNarrowPhase/TATCollisionUtil.h"
#include "../TATDynamics/TATRigidBody.h"
#include "../TATPositionBasedDynamics/TATPBDBody.h"

#define FACE_EXTENT 0.1

struct ConvexFacePack
{
	int m_FaceIndex;
	TATVector3 m_Pos[3];
	TATVector3 m_PredictPos[3];
	TATVector3 m_Vel[3];
};

class SoftConvexFaceOverlapCallBack :public TATBvhCollideCallBack
{
public:
	SoftConvexFaceOverlapCallBack(TATRigidBody* rb, std::vector<TATSoftRigidCollideData>& collideDatas,float dt) :
		m_Rigid(rb),m_CollideDatas(collideDatas), m_DeltaTime(dt)
	{

	}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
	{
		TATPBDParticle* particle = (TATPBDParticle*)(node1->m_Data);
		ConvexFacePack* facePack = (ConvexFacePack*)(node2->m_Data);

		TATVector3 vel = particle->m_PredictPos - particle->Position();

		float t;
		float margin = 0.2;
		float collideRadius = 0.02f;

		if (TATCollisionUtil::CalcTimeOfImpact
		(
			particle->Position(),
			particle->m_PredictPos - particle->Position(),
			m_Rigid,
			m_Rigid->GetLinearVelocity(),
			m_Rigid->GetAngularVelocity(),
			facePack->m_FaceIndex,
			m_DeltaTime,
			margin,
			collideRadius,
			64,
			t
		))
		{
			float time = t * m_DeltaTime;
			TATransform predictTr;
			TATransformUtil::IntegrateTransform
			(
				m_Rigid->GetWorldTransform(),
				m_Rigid->GetLinearVelocity(),
				m_Rigid->GetAngularVelocity(),
				time,
				predictTr
			);
			TATCollideShapeConvex* convex = m_Rigid->m_CollideShape->Cast<TATCollideShapeConvex>();
			const TATPhyFace& face = convex->m_CollideMeshData.m_Faces[facePack->m_FaceIndex];
			TATVector3 points[3]
			{
				predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[0]].m_Position,
				predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[1]].m_Position,
				predictTr * convex->m_CollideMeshData.m_Vertices[face.m_VertexIndices[2]].m_Position
			};
			TATVector3 norm = (points[1] - points[0]).Cross(points[2] - points[0]).Normalized();
			TATVector3 p = (particle->m_PredictPos - particle->Position()) * t + particle->Position();
			float dist = (p - points[0]).Dot(norm) - margin;

			float pen = collideRadius - dist;

			TATSoftRigidCollideData data;
			data.m_Soft = particle->m_HostBody;
			data.m_Rigid = m_Rigid;
			data.m_Particle = particle;
			data.m_CollideNormal = norm;
			data.m_Penetration = pen;
			data.m_RigidPt = p - norm * (dist + margin);
			data.m_SoftPt = p;
			data.m_HitFraction = t;

			m_CollideDatas.push_back(data);
		}

		/*if (TATCollisionUtil::CalcTimeOfImpact(particle->Position(), vel, facePack->m_Pos, facePack->m_Vel, t, 10, margin))
		{
			TATVector3 norm = ((facePack->m_Pos[1] - facePack->m_Pos[0]).Cross(facePack->m_Pos[2] - facePack->m_Pos[0])).Normalized();

			TATVector3 p = particle->Position() + vel * t;

			float dist = (p - facePack->m_Pos[0]).Dot(norm);

			float pen = margin + COLLIDE_EPS - (p - facePack->m_Pos[0]).Dot(norm);

			float pen1 = margin + COLLIDE_EPS - (particle->m_PredictPos - facePack->m_Pos[0]).Dot(norm);

			float pen2 = (vel * (1 - t)).Dot(norm);

			TATVector3 normal = ((facePack->m_PredictPos[1] - facePack->m_PredictPos[0]).
				Cross(facePack->m_PredictPos[2] - facePack->m_PredictPos[0])).Normalized();
			float pen3 = margin + COLLIDE_EPS - (particle->m_PredictPos - facePack->m_PredictPos[0]).Dot(normal);

			TATSoftRigidCollideData data;
			data.m_CollideNormal = norm;
			data.m_Particle = particle;
			data.m_Rigid = m_Rigid;

			data.m_Penetration = pen3;
			data.m_RigidPt = p - norm * dist;
			data.m_SoftPt = p + norm * COLLIDE_EPS;

			m_CollideDatas.push_back(data);
		}*/
	}

	TATRigidBody* m_Rigid;
	std::vector<TATSoftRigidCollideData>& m_CollideDatas;
	float m_DeltaTime;
};

class TATSoftCollideConvexAlgo : public TATSoftRigidCollisionAlgoPrimitive
{
public:
	virtual bool ProcessSoftRigidCollision(TATPBDBody* soft, TATRigidBody* rb, float dt)
	{
		TATCollideShapeConvex* convex = dynamic_cast<TATCollideShapeConvex*>(rb->m_CollideShape);

		TATBvh faceTree;
		TATVector3 min, max;

		const TATransform& tr = rb->GetWorldTransform();

		std::vector<TATPhyFace>& faces = convex->m_CollideMeshData.m_Faces;
		std::vector<TATPhyVertex>& vertices = convex->m_CollideMeshData.m_Vertices;

		TATVector3 pos0, pos1, pos2;
		TATVector3 vel0, vel1, vel2;
		TATVector3 predict0, predict1, predict2;

		std::vector<ConvexFacePack> facePack;
		facePack.resize(faces.size());
		for (int i = 0; i < faces.size(); ++i)
		{
			const TATPhyVertex& vertex0 = vertices[faces[i].m_VertexIndices[0]];
			const TATPhyVertex& vertex1 = vertices[faces[i].m_VertexIndices[1]];
			const TATPhyVertex& vertex2 = vertices[faces[i].m_VertexIndices[2]];

			pos0 = tr * vertex0.m_Position;
			pos1 = tr * vertex1.m_Position;
			pos2 = tr * vertex2.m_Position;

			vel0 = rb->GetVelocityAtWCS(pos0);
			vel1 = rb->GetVelocityAtWCS(pos1);
			vel2 = rb->GetVelocityAtWCS(pos2);

			predict0 = pos0 + vel0 * dt;
			predict1 = pos1 + vel1 * dt;
			predict2 = pos2 + vel2 * dt;

			min = pos0;
			min.SetMin(pos1);
			min.SetMin(pos2);
			min.SetMin(predict0); min.SetMin(predict1); min.SetMin(predict2);

			max = pos0;
			max.SetMax(pos1);
			max.SetMax(pos2);
			max.SetMax(predict0); max.SetMax(predict1); max.SetMax(predict2);

			facePack[i].m_FaceIndex = i;
			facePack[i].m_Pos[0] = pos0; facePack[i].m_Pos[1] = pos1; facePack[i].m_Pos[2] = pos2;
			facePack[i].m_PredictPos[0] = predict0;
			facePack[i].m_PredictPos[1] = predict1;
			facePack[i].m_PredictPos[2] = predict2;
			facePack[i].m_Vel[0] = predict0 - facePack[i].m_Pos[0];
			facePack[i].m_Vel[1] = predict1 - facePack[i].m_Pos[1];
			facePack[i].m_Vel[2] = predict2 - facePack[i].m_Pos[2];

			faceTree.InsertAabbNode(min - TATVector3::One() * FACE_EXTENT, max + TATVector3::One() * FACE_EXTENT)->m_Data = (void*)(&facePack[i]);
		}

		faceTree.FinishBuild();

		SoftConvexFaceOverlapCallBack cb(rb, m_CollisionDatas, dt);
		faceTree.CollideWithBVTree(&soft->m_ParticleBVH, &cb);

		if (m_CollisionDatas.size() > 0)
			return true;
		else
			return false;
	}
};
