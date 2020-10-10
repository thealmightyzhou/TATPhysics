#pragma once
#include "../TATGeometry/TATMeshInfo.h"
#include "../TATCommon/TATransform.h"
#include "../TATCommon/TATAabb.h"
#include "../TATCommon/TATObjectPool.h"
#include "../TATBasis/TATickable.h"
#include "TATPhyBody.h"

class TATActor;

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
		m_Margin = 0.05f;
	}
	virtual ~TATCollideShapePrimitive() {}

	float GetAngularMotionDisc()
	{
		TATVector3 centerLocal;
		float radius;
		m_LocalAabb.GetLocalBoundingSphere(centerLocal, radius);
		return centerLocal.Length() + radius;
	}

	template<typename T>
	T* Cast()
	{
		return dynamic_cast<T*>(this);
	}

	TAT_REGISTER_ATTRIBUTE_GET(CollideShapeType, ShapeType);

	float m_InvMass;

	float m_OriginMass;

	float m_Margin;

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
	TATCollideShapeConvex(const TATPhyMeshData& meshData, float invMass, bool autofill = true);

	TATPhyMeshData m_CollideMeshData;

};

class TATCollideShapePlane : public TATCollideShapePrimitive
{
public:
	TATCollideShapePlane(const TATVector3& origin, const TATVector3& normal)
		:TATCollideShapePrimitive(0), m_Origin(origin), m_Normal(normal)
	{
		m_LocalInertiaTensor = TATMatrix3::GetZero();
		m_LocalInvInertiaTensor = TATMatrix3::GetZero();
		m_LocalMassCenter = m_Origin;
		m_LocalAabb.SetOrigin(-TAT_MAXVECTOR3, TAT_MAXVECTOR3);
		m_ShapeType = CollideShapeType::CollidePlane;
	}

	TATVector3 m_Origin;
	TATVector3 m_Normal;
};

class TATRigidBody : public TATickable ,public TATPhyBody
{
public:
	TATRigidBody(const TString& name,TATCollideShapePrimitive* cs) :m_CollideShape(cs),TATickable(name + TString::ConvertInt(GetObjectIndex())),TATPhyBody(BodyType::RigidBody)
	{
		SetWorldTransform(TATransform::GetIdentity());

		m_FrictionCoeff = 0.9;
		m_ContactHardness = 1;
		m_LinFactor.SetValue(1, 1, 1);
		m_AngFactor.SetValue(1, 1, 1);
		m_UseCCD = false;
		m_ControllByUser = false;
	}

	TATRigidBody(const TString& name,TATCollideShapePrimitive* cs, const TATransform& tr) :m_CollideShape(cs), m_WorldTransform(tr),
		TATickable(name + TString::ConvertInt(GetObjectIndex())), TATPhyBody(BodyType::RigidBody)
	{
		m_FrictionCoeff = 0.9;
		m_ContactHardness = 1;
		m_LinFactor.SetValue(1, 1, 1);
		m_AngFactor.SetValue(1, 1, 1);
		m_UseCCD = false;
		m_ControllByUser = false;
	}

	virtual bool Update(TATActor* actor, float dt);

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

	TATVector3 GetMassCenter() const
	{
		return  m_WorldTransform * m_CollideShape->m_LocalMassCenter;
	}

	TATVector3 GetVelocityAtWCS(const TATVector3& location) const;

	TATVector3 GetVelocityAtLCS(const TATVector3& location) const;

	TATVector3 GetLinearVelocity() const;

	TATVector3 GetAngularVelocity() const;

	TATVector3 GetVelocityAt(const TATVector3& r)
	{
		return m_LinVel + m_AngVel.Cross(r);
	}

	void UpdataInverseInertiaWorld();

	void ApplyImpulse(const TATVector3& impulse, const TATVector3& r);

	TAT_POOL_OBJECT(TATRigidBody);

	TAT_REGISTER_ATTRIBUTE(TATransform, WorldTransform);

	TATransform m_PreWorldTransform;

	TATCollideShapePrimitive* m_CollideShape;

	float m_InvMass;

	float m_ContactHardness;

	float m_RestituitionCoeff;

	float m_FrictionCoeff;
	
	int m_BodyIndex;

	bool m_UseCCD;

	//do not update by dynamic world
	bool m_ControllByUser;

	TATVector3 m_LinFactor;

	TATVector3 m_AngFactor;

	TATVector3 m_Pos;

	TATQuaternion m_Quat;

	TATVector3 m_LinVel;

	TATVector3 m_AngVel;

	TATVector3 m_Gravity;

	TATMatrix3 m_InvInertiaWorld;

	TATMatrix3 m_InitInvInertia;

};

