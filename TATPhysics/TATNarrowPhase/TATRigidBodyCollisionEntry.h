#pragma once
#include "TATRigidBodyCollideData.h"
#include "TATCCD.h"

class TATRbCollision
{
public:

	static bool Collide(TATRigidBody* rb0, TATRigidBody* rb1, TATRigidBodyCollideData& data)
	{

		unsigned long mask = 0;

		CollideShapeType type0 = rb0->GetShapeType();
		CollideShapeType type1 = rb1->GetShapeType();
		mask |= (1 << type0);
		mask |= (1 << type1);

		const TATransform tr0 = rb0->GetWorldTransform();
		const TATransform tr1 = rb1->GetWorldTransform();

		TATCollideShapePrimitive* cs0 = rb0->m_CollideShape;
		TATCollideShapePrimitive* cs1 = rb1->m_CollideShape;

		if (mask & (1 << CollideShapeType::CollideConvex))
		{
			mask &= ~(1 << CollideShapeType::CollideConvex);

			if (mask & (1 << CollideShapeType::CollideConvex))
			{
				TATSATCollideData satdata;
				bool collide = TATSAT::SeparateAxisTest(TATRigidBodyGroup(rb0, rb1), satdata);
				if (collide)
				{
					data.m_CollideNormal = satdata.m_CollideNormal;
					data.m_CollidePt0 = satdata.m_CollidePtA;
					data.m_CollidePt1 = satdata.m_CollidePtB;
					data.m_Penetration = satdata.m_Penetration;
					data.m_RbIndex0 = rb0->m_IndexInPool;
					data.m_RbIndex1 = rb1->m_IndexInPool;
				}
				else
					return false;

			}
			else if (mask & (1 << CollideShapeType::CollidePlane))
			{

			}
		}
	}
};