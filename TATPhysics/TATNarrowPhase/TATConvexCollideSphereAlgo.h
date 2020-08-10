#pragma once
#pragma once
#include "TATRigidBodyCollideData.h"
#include "../TATCommon/TATRange.h"

class TATConvexCollideSphereAlgo :public TATCollideAlgoPrimitive
{
	bool ComputeCollision(TATRigidBody* rbConvex, TATRigidBody* rbSphere) override
	{
		CollideShapeType type0 = rbConvex->GetShapeType();
		CollideShapeType type1 = rbSphere->GetShapeType();

		if (type0 == CollideShapeType::CollideSphere && type1 == CollideShapeType::CollideConvex)
			std::swap(rbConvex, rbSphere);

		TATCollideShapeConvex* convex = dynamic_cast<TATCollideShapeConvex*>(rbConvex->m_CollideShape);
		TATCollideShapeSphere* sphere = dynamic_cast<TATCollideShapeSphere*>(rbSphere->m_CollideShape);
		const TATransform& tr0 = rbConvex->GetWorldTransform();
		const TATransform& tr1 = rbSphere->GetWorldTransform();

		if (!convex || !sphere)
			return false;

		float penetration = TAT_MAX;
		TATVector3 collideNormal;
		TATVector3 collidePtOnConvex;

		float centerProj;
		float sphereProjMin, sphereProjMax;
		int min, max;
		float proj0, proj1;
		
		TATVector3 ct = tr1 * sphere->m_Center;
		TATVector3 dirCorrect = tr0 * convex->m_LocalMassCenter - ct;
		for (int i = 0; i < convex->m_CollideMeshData.m_Faces.size(); ++i)
		{
			const TATPhyFace& face = convex->m_CollideMeshData.m_Faces[i];

			TATVector3 dir = tr0.GetBasis() * face.GetNormal();
			if (dir.Dot(dirCorrect) < 0)
				dir = -dir;

			Support(convex, tr0, dir, min, max, proj0, proj1);

			centerProj = ct.Dot(dir);
			sphereProjMin = centerProj - sphere->m_Radius;
			sphereProjMax = centerProj + sphere->m_Radius;

			TATRange rg0(proj0, proj1);

			TATRange rg1(sphereProjMin, sphereProjMax);

			float pen = rg0.Penetration(rg1);
			if (pen < 0)
				return false;
			else if (pen < penetration)
			{
				penetration = pen;
				collideNormal = dir;
			}

		}

		m_CollideData.m_Penetration = penetration;
		m_CollideData.m_CollidePt1 = ct + collideNormal * sphere->m_Radius;
		m_CollideData.m_CollidePt0 = m_CollideData.m_CollidePt1 - collideNormal * penetration;
		m_CollideData.m_CollideNormal = collideNormal;

		m_CollideData.m_RbIndex0 = rbConvex->m_IndexInPool;
		m_CollideData.m_RbIndex1 = rbSphere->m_IndexInPool;
	}
};