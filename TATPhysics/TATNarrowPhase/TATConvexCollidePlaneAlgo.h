#pragma once
#include "TATRigidBodyCollideData.h"

class TATConvexCollidePlaneAlgo:public TATCollideAlgoPrimitive
{
	//default can collide double direction on plane
	bool ComputeCollision(TATRigidBody* rbConvex, TATRigidBody* rbPlane) override
	{
		TATCollideShapeConvex* convex = dynamic_cast<TATCollideShapeConvex*>(rbConvex->m_CollideShape);
		TATCollideShapePlane* plane = dynamic_cast<TATCollideShapePlane*>(rbPlane->m_CollideShape);
		TATransform tr = rbConvex->GetWorldTransform();
		if (!convex || !plane)
			return false;

		float planeProj = plane->m_Origin.Dot(plane->m_Normal);
		int minId, maxId;
		float minProj, maxProj;

		Support(convex, tr, plane->m_Normal, minId, maxId, minProj, maxProj);

		if (planeProj > maxProj || planeProj < minProj)
		{
			return false;
		}

		float dist0 = planeProj - minProj;
		float dist1 = maxProj - planeProj;

		int whichPoint = 0;

		if (dist0 < dist1)
			m_CollideData.m_CollidePt0 = tr * convex->m_CollideMeshData.m_Vertices[minId].m_Position;
		else
		{
			m_CollideData.m_CollidePt0 = tr * convex->m_CollideMeshData.m_Vertices[maxId].m_Position;
			whichPoint = 1;
		}

		TATVector3 dir;
		if ((m_CollideData.m_CollidePt0 - plane->m_Origin).Dot(plane->m_Normal) < 0)
			dir = -plane->m_Normal;
		else
			dir = plane->m_Normal;

		m_CollideData.m_Penetration = whichPoint == 0 ? dist0 : dist1;
		m_CollideData.m_CollidePt1 = m_CollideData.m_CollidePt0 - dir * m_CollideData.m_Penetration;

		m_CollideData.m_RbIndex0 = rbConvex->m_IndexInPool;
		m_CollideData.m_RbIndex1 = rbPlane->m_IndexInPool;
		m_CollideData.m_CollideNormal = dir;
	}
};