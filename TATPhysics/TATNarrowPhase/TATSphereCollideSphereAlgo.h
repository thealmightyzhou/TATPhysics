#pragma once
#include "TATRigidBodyCollideData.h"

class TATRigidBodyCollideData :public TATCollideAlgoPrimitive
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

		if (dist > sphere0->m_Radius + sphere1->m_Radius)
			return false;

		
		m_CollideData.m_Penetration = sphere0->m_Radius + sphere1->m_Radius - dist;
		//TODO
	}
};