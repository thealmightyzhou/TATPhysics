#pragma once
#include "../TATPositionBasedDynamics/TATPBDBody.h"
#include "../TATDynamics/TATRigidBody.h"
#include "TATSoftCollidePlaneAlgo.h"
#include "TATSoftCollideConvexAlgo.h"

class TATSoftRigidCollisionEntry
{
public:
	static bool ProcessSoftRigidCollision(TATPBDBody* soft, TATRigidBody* rb, float dt, std::vector<TATSoftRigidCollideData>& data)
	{
		TATSoftRigidCollisionAlgoPrimitive* algo = GetAlgo(soft, rb);

		if (algo)
		{
			bool collide = algo->ProcessSoftRigidCollision(soft, rb, dt);
			if (collide)
			{
				data.insert(data.end(), algo->m_CollisionDatas.begin(), algo->m_CollisionDatas.end());
			}

			delete algo;
			algo = 0;

			return collide;
		}

		return false;
	}

	static void SolveRSContacts(std::vector<TATSoftRigidCollideData>& data, float dt)
	{
		TATSoftRigidCollisionAlgoPrimitive* algo = 0;
		for (int i = 0; i < data.size(); ++i)
		{
			algo = GetAlgo(data[i].m_Soft, data[i].m_Rigid);
			if (algo)
			{
				algo->SolveRSContacts(data[i],dt);
				delete algo;
				algo = 0;
			}
		}
	}

	static TATSoftRigidCollisionAlgoPrimitive* GetAlgo(TATPBDBody* soft, TATRigidBody* rb)
	{
		TATSoftRigidCollisionAlgoPrimitive* algo = 0;
		switch (rb->GetShapeType())
		{
		case CollideShapeType::CollidePlane:
			algo = new TATSoftCollidePlaneAlgo();
			break;
		case CollideShapeType::CollideConvex:
			algo = new TATSoftCollideConvexAlgo();
			break;
		}
	}
};
