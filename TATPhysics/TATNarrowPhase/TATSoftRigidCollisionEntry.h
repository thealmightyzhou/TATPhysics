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
};
