#pragma once
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATCore.h"
#include "../TATDynamics/TATRigidBody.h"

//in wcs
struct TATRigidBodyCollideData
{
public:

	int m_RbIndex0; //get from TATDynamicWorld
	int m_RbIndex1;
	float m_Penetration;
	TATVector3 m_CollidePt0;
	TATVector3 m_CollidePt1;
	TATVector3 m_CollideNormal; //rb1 -> rb0
	float m_FrictionCoefficent;
	float m_RestitutionCoefficent;
	float m_RollingFrictionCoefficent;

	void SetFrictionCoeff(float f) //0 - 1
	{
		m_FrictionCoefficent = (unsigned short)(f * 0xffff);
	}

	float GetFrictionCoeff() const
	{
		return ((float)m_FrictionCoefficent / (float)0xffff);
	}
};

class TATCollideAlgoPrimitive
{
public:
	virtual ~TATCollideAlgoPrimitive() {}

	void Support(TATCollideShapeConvex* convex, const TATransform& tr, const TATVector3& dir,
		int& min, int& max, float& proj0, float& proj1)
	{
		const std::vector<TATPhyVertex>& vertices = convex->m_CollideMeshData.m_Vertices;

		proj0 = TAT_MAX;
		proj1 = -TAT_MAX;
		for (int i = 0; i < vertices.size(); ++i)
		{
			float proj = (tr * vertices[i].m_Position).Dot(dir);
			if (proj < proj0)
			{
				proj0 = proj;
				min = i;
			}
			if (proj > proj1)
			{
				proj1 = proj;
				max = i;
			}

		}
	}

	virtual bool ComputeCollision(TATRigidBody* rb0, TATRigidBody* rb1)
	{
		return false;
	}

	TATRigidBodyCollideData m_CollideData;

};

