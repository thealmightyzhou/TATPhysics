#pragma once
#include "TATRigidBodyCollideData.h"
#include "TATSAT.h"

class TATConvexCollideConvexAlgo :public TATCollideAlgoPrimitive
{
public:
	//use sat or gjk
	bool ComputeCollision(TATRigidBody* rb0,TATRigidBody* rb1) override
	{
		TATSATCollideData satdata;
		bool collide = TATSAT::SeparateAxisTest(TATRigidBodyGroup(rb0, rb1), satdata);
		if (collide)
		{
			m_CollideData.m_CollideNormal = satdata.m_CollideNormal;
			m_CollideData.m_CollidePt0 = satdata.m_CollidePtA;
			m_CollideData.m_CollidePt1 = satdata.m_CollidePtB;
			m_CollideData.m_Penetration = satdata.m_Penetration;
			m_CollideData.m_RbIndex0 = rb0->m_IndexInPool;
			m_CollideData.m_RbIndex1 = rb1->m_IndexInPool;
			m_CollideData.SetFrictionCoeff(0.7);
		}

		return collide;
	}
};