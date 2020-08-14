#pragma once
#pragma once
#include "TATRigidBodyCollideData.h"

class TATSphereCollidePlaneAlgo :public TATCollideAlgoPrimitive
{
	bool ComputeCollision(TATRigidBody* rbSphere, TATRigidBody* rbPlane) override
	{
		CollideShapeType shape0 = rbSphere->GetShapeType();
		CollideShapeType shape1 = rbPlane->GetShapeType();

		if (shape0 == CollideShapeType::CollidePlane && shape1 == CollideShapeType::CollideSphere)
			std::swap(rbSphere, rbPlane);

		TATCollideShapeSphere* sphere = dynamic_cast<TATCollideShapeSphere*>(rbSphere->m_CollideShape);
		TATCollideShapePlane* plane = dynamic_cast<TATCollideShapePlane*>(rbPlane->m_CollideShape);
		const TATransform& tr = rbSphere->GetWorldTransform();

		const TATVector3& ct = tr * sphere->m_Center;

		float dist = abs((ct - plane->m_Origin).Dot(plane->m_Normal));
		if (dist > sphere->m_Radius)
			return false;

		TATVector3 dir = (ct - plane->m_Origin).Normalized();
		if (dir.Dot(plane->m_Normal) < 0)
			dir = -plane->m_Normal;
		else
			dir = plane->m_Normal;

		m_CollideData.m_Penetration = sphere->m_Radius - dist;
		m_CollideData.m_CollidePt0 = ct - dir * sphere->m_Radius;
		m_CollideData.m_CollidePt1 = ct - dir * dist;
		m_CollideData.m_CollideNormal = dir;
		m_CollideData.m_RbIndex0 = rbSphere->m_IndexInPool;
		m_CollideData.m_RbIndex1 = rbPlane->m_IndexInPool;
		m_CollideData.SetFrictionCoeff(0.7);
	}
};