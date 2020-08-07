#pragma once
#include "TATRigidBodyCollideData.h"

class TATSphereCollideSphereAlgo :public TATCollideAlgoPrimitive
{
	bool ComputeCollision(TATRigidBody* rbSphere0, TATRigidBody* rbSphere1) override
	{
		TATCollideShapeSphere* sphere0 = dynamic_cast<TATCollideShapeSphere*>(rbSphere0->m_CollideShape);
		TATCollideShapeSphere* sphere1 = dynamic_cast<TATCollideShapeSphere*>(rbSphere1->m_CollideShape);
		const TATransform& tr0 = rbSphere0->GetWorldTransform();
		const TATransform& tr1 = rbSphere1->GetWorldTransform();

		const TATVector3& ct0 = tr0 * sphere0->m_Center;
		const TATVector3& ct1 = tr1 * sphere1->m_Center;

		float dist = ct0.Distance(ct1);

		if (dist > (sphere0->m_Radius + sphere1->m_Radius) || dist < abs(sphere0->m_Radius - sphere1->m_Radius))
			return false;
		
		TATVector3 dir = (ct1 - ct0).Normalized();

		m_CollideData.m_Penetration = sphere0->m_Radius + sphere1->m_Radius - dist;
		m_CollideData.m_CollidePt0 = ct0 + dir * sphere0->m_Radius;
		m_CollideData.m_CollidePt1 = ct1 - dir * sphere1->m_Radius;
		m_CollideData.m_CollideNormal = dir;
		m_CollideData.m_RbIndex0 = rbSphere0->m_IndexInPool;
		m_CollideData.m_RbIndex1 = rbSphere1->m_IndexInPool;
	}
};