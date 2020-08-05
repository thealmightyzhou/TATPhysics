#pragma once
#include "../TATGeometry/TATMeshInfo.h"
#include "../TATCommon/TATransform.h"
#include "../TATCommon/TATAabb.h"
#include "../TATCommon/TATObjectPool.h"

enum CollideShapeType
{
	CollideNULL,
	CollideSphere,
	CollideCuboid,
	CollideCapsule,
	CollideCylinder,
	CollideConvex,
	CollideCompound,
	CollidePlane,
};

class TATCollideShapePrimitive
{
public:
	TATCollideShapePrimitive(float invMass) :m_InvMass(invMass)
	{
		m_LocalMassCenter = TATVector3::Zero();
		m_LocalInertiaTensor = TATMatrix3::GetIdentity();
		m_LocalInvInertiaTensor = TATMatrix3::GetIdentity();
	}
	virtual ~TATCollideShapePrimitive() {}

	float GetAngularMotionDisc()
	{
		TATVector3 centerLocal;
		float radius;
		m_LocalAabb.GetLocalBoundingSphere(centerLocal, radius);
		return centerLocal.Length() + radius;
	}

	TAT_REGISTER_ATTRIBUTE_GET(CollideShapeType, ShapeType);

	float m_InvMass;

	float m_OriginMass;

	TATVector3 m_LocalMassCenter;

	TATMatrix3 m_LocalInertiaTensor;

	TATMatrix3 m_LocalInvInertiaTensor;

	TATAabb m_LocalAabb;
};

class TATCollideShapeSphere : public TATCollideShapePrimitive
{
public:
	TATCollideShapeSphere(const TATVector3& center, float radius, float invMass);
	TATVector3 m_Center;
	float m_Radius;

};

class TATCollideShapeCuboid : public TATCollideShapePrimitive
{
public:
	TATCollideShapeCuboid(const TATVector3& center, const TATVector3& extend, float invMass);
	TATVector3 m_Center;
	TATVector3 m_Extend;
};

class TATCollideShapeCylinder :public TATCollideShapePrimitive
{
public:
	TATVector3 m_Center;
	TATVector3 m_Radius;
	TATVector3 m_Top;
	TATVector3 m_Bottom;
};

class TATCollideShapeConvex :public TATCollideShapePrimitive
{
public:
	TATCollideShapeConvex(const TATPhyMeshData& meshData, float invMass);

	TATPhyMeshData m_CollideMeshData;

};

class TATCollideShapePlane : public TATCollideShapePrimitive
{
public:
	TATCollideShapePlane(const TATVector3& origin, const TATVector3& normal)
		:TATCollideShapePrimitive(0), m_Origin(origin), m_Normal(normal)
	{
		m_LocalInertiaTensor = TATMatrix3::GetIdentity();
		m_LocalMassCenter = m_Origin;
		m_LocalAabb.SetOrigin(-TAT_MAXVECTOR3, TAT_MAXVECTOR3);
		m_ShapeType = CollideShapeType::CollidePlane;
	}

	TATVector3 m_Origin;
	TATVector3 m_Normal;
};

class TATRigidBody
{
public:
	TATRigidBody(TATCollideShapePrimitive* cs) :m_CollideShape(cs)
	{
		SetWorldTransform(TATransform::GetIdentity());
	}

	TATRigidBody(TATCollideShapePrimitive* cs, const TATransform& tr) :m_CollideShape(cs), m_WorldTransform(tr)
	{

	}

	TAT_POOL_OBJECT(TATRigidBody);

	TAT_REGISTER_ATTRIBUTE(TATransform, WorldTransform);

	CollideShapeType GetShapeType() 
	{ 
		if (m_CollideShape)
			return m_CollideShape->GetShapeType();
		else
			return CollideShapeType::CollideNULL;
	}

	void UpdateWorldAabb() 
	{
		m_CollideShape->m_LocalAabb.Update(m_WorldTransform);
	}

	void GetWorldAabb(TATVector3& min, TATVector3& max)
	{
		m_CollideShape->m_LocalAabb.GetWorldBoundingAabb(min, max);
	}

	void SetMass(float m)
	{
		if (m <= TAT_EPSILON2)
			m_InvMass = 0.0f;
		else
			m_InvMass = float(1) / m;
	}

	void SetCollideShape(TATCollideShapePrimitive* cs)
	{
		m_CollideShape = cs;
	}

	TATVector3 GetMassCenter()
	{
		return  m_WorldTransform * m_CollideShape->m_LocalMassCenter;
	}

	TATCollideShapePrimitive* m_CollideShape;

	float m_InvMass;
	
	int m_BodyIndex;

	int m_DataIndex;

	int m_InertiaIndex;
};

