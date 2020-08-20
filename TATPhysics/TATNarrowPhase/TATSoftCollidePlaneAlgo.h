#pragma once
#include "TATSoftRigidCollisionData.h"
#include "../TATBroadPhase/TATBvh.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"
#include "../TATNarrowPhase/TATCollisionUtil.h"
#include "../TATDynamics/TATRigidBody.h"
#include "../TATPositionBasedDynamics/TATPBDBody.h"

class SoftPlaneOverlapCallBack :public TATBvhCollideCallBack
{
public:
	SoftPlaneOverlapCallBack(TATVector3 points[4], TATRigidBody* rb, std::vector<TATSoftRigidCollideData>& collideDatas) :
		m_Rigid(rb), m_CollideDatas(collideDatas)
	{
		for (int i = 0; i < 4; ++i)
		{
			m_PlanePoints[i] = points[i];
		}
	}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
	{
		TATPBDParticle* particle = (TATPBDParticle*)(node1->m_Data);
		int which = (int)(node2->m_Data);

		TATVector3 points[3];
		if (which == 0)
		{
			points[0] = m_PlanePoints[0];
			points[1] = m_PlanePoints[1];
			points[2] = m_PlanePoints[2];
		}
		else
		{
			points[0] = m_PlanePoints[0];
			points[1] = m_PlanePoints[2];
			points[2] = m_PlanePoints[3];
		}

		TATVector3 faceVel[3]{ TATVector3::Zero(),TATVector3::Zero() ,TATVector3::Zero() };
		TATVector3 vel = particle->m_PredictPos - particle->Position();

		float t;
		float margin = 0.2;

		if (TATCollisionUtil::CalcTimeOfImpact(particle->Position(), vel, points, faceVel, t, 10, margin))
		{
			TATVector3 p = particle->Position() + vel * t;
			//plane face is static
			TATVector3 norm = ((points[1] - points[0]).Cross(points[2] - points[0])).Normalized();
			float pen = margin - (p - m_PlanePoints[0]).Dot(norm);

			TATSoftRigidCollideData data;
			data.m_CollideNormal = norm;
			data.m_Particle = particle;
			data.m_Rigid = m_Rigid;
			data.m_Penetration = pen;
			data.m_RigidPt = p - norm * pen;
			data.m_SoftPt = p + norm * COLLIDE_EPS;

			m_CollideDatas.push_back(data);
		}
	}

	TATVector3 m_PlanePoints[4];
	TATRigidBody* m_Rigid;
	std::vector<TATSoftRigidCollideData>& m_CollideDatas;
};

class TATSoftCollidePlaneAlgo : public TATSoftRigidCollisionAlgoPrimitive
{
public:
	virtual bool ProcessSoftRigidCollision(TATPBDBody* soft, TATRigidBody* rb, float dt)
	{
		TATCollideShapePlane* colPlane = dynamic_cast<TATCollideShapePlane*>(rb->m_CollideShape);
		TATVector3 l1, l2;
		colPlane->m_Normal.PlaneSpace(l1, l2);

		TATVector3 points[4];
		points[0] = colPlane->m_Origin + l1 * 10000;
		points[1] = colPlane->m_Origin + l2 * 10000;
		points[2] = colPlane->m_Origin - l1 * 10000;
		points[3] = colPlane->m_Origin - l2 * 10000;
		//0,1,2 ; 0,2,3

		TATBvh planeTree;
		TATVector3 min = points[0];
		min.SetMin(points[1]);
		min.SetMin(points[2]);
		TATVector3 max = points[0];
		max.SetMax(points[1]);
		max.SetMax(points[2]);

		TATBVNode* node = planeTree.InsertAabbNode(min - TATVector3::One() * 0.5f, max + TATVector3::One() * 0.5f);
		node->m_Data = (void*)(0); //if 0 take 0,1,2

		min = points[0];
		min.SetMin(points[2]);
		min.SetMin(points[3]);
		max = points[0];
		max.SetMax(points[2]);
		max.SetMax(points[3]);
		node = planeTree.InsertAabbNode(min - TATVector3::One() * 0.5f, max + TATVector3::One() * 0.5f);
		node->m_Data = (void*)(1);

		planeTree.FinishBuild();

		SoftPlaneOverlapCallBack cb(points, rb, m_CollisionDatas);
		planeTree.CollideWithBVTree(&soft->m_ParticleBVH, &cb);

		if (m_CollisionDatas.size() > 0)
			return true;
		else
			return false;
	}
};

