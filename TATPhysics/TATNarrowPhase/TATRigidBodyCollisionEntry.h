#pragma once
#include "TATConvexCollideConvexAlgo.h"
#include "TATConvexCollidePlaneAlgo.h"
#include "TATConvexCollideSphereAlgo.h"
#include "TATSphereCollideSphereAlgo.h"
#include "TATSphereCollidePlaneAlgo.h"

class TATRbCollision
{
public:

	static bool RbCollision(TATRigidBody* rb0, TATRigidBody* rb1, TATRigidBodyCollideData& data)
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

		TATCollideAlgoPrimitive* algo = 0;

		if (mask & (1 << CollideShapeType::CollideConvex))
		{
			mask &= ~(1 << CollideShapeType::CollideConvex); //0001000100 &= 1110111111 => 0000000100

			if (mask == 0) //both are convex shape
				algo = new TATConvexCollideConvexAlgo;

			else if (mask & (1 << CollideShapeType::CollidePlane))
				algo = new TATConvexCollidePlaneAlgo;

			else if (mask & (1 << CollideShapeType::CollideSphere))
				algo = new TATConvexCollideSphereAlgo;
		}

		else if (mask & (1 << CollideShapeType::CollideSphere))
		{
			mask &= ~(1 << CollideShapeType::CollideSphere);

			if (mask == 0)
				algo = new TATSphereCollideSphereAlgo;

			else if (mask & (1 << CollideShapeType::CollidePlane))
				algo = new TATSphereCollidePlaneAlgo;

		}

		//---------------------
		
		if (algo)
		{
			if (algo->ComputeCollision(rb0, rb1))
			{
				data = algo->m_CollideData;

				delete algo;
				return true;
			}

			delete algo;
			return false;

		}

		return false;

	}
};