#include "TATRigidBody.h"
#include "../TATGeometry/TATInertia.h"
#include "../TATStage/TATActor.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "TATDynamicWorld.h"
#include "../TATCommon/TATObjectPool.h"

TATCollideShapeSphere::TATCollideShapeSphere(const TATVector3& center, float radius, float invMass)
	:m_Center(center), m_Radius(radius), TATCollideShapePrimitive(invMass)
{
	m_ShapeType = CollideShapeType::CollideSphere;
	m_LocalMassCenter = m_Center;
	m_LocalInertiaTensor = TATInertiaComputer::ComputeInertia(this);
	m_LocalInvInertiaTensor = m_LocalInertiaTensor.Inverse();
	m_LocalAabb.SetOriginSphere(center, radius);
}

//=======================

TATCollideShapeCuboid::TATCollideShapeCuboid(const TATVector3& center, const TATVector3& extend, float invMass)
	:m_Center(center), m_Extend(extend), TATCollideShapePrimitive(invMass)
{
	m_ShapeType = CollideShapeType::CollideCuboid;
	m_LocalMassCenter = m_Center;
	m_LocalInertiaTensor = TATInertiaComputer::ComputeInertia(this);
	m_LocalInvInertiaTensor = m_LocalInertiaTensor.Inverse();
	m_LocalAabb.SetOriginExtend(center, extend);
}

//========================
//will automatically fill the physic infomation
TATCollideShapeConvex::TATCollideShapeConvex(const TATPhyMeshData& meshData,float invMass)
	:m_CollideMeshData(meshData), TATCollideShapePrimitive(invMass)
{
	m_ShapeType = CollideShapeType::CollideConvex;
	TATVector3 min, max;
	m_LocalMassCenter = TATPhyMeshDataComputer::ComputeMassCentreAndLocalAabb(m_CollideMeshData.m_Vertices, min, max);
	m_LocalAabb.SetOrigin(min, max);
	m_LocalInertiaTensor = TATInertiaComputer::ComputeInertia(this);
	m_LocalInvInertiaTensor = m_LocalInertiaTensor.Inverse();
	TATPhyMeshDataComputer::CompleteMeshData(m_CollideMeshData.m_Vertices, m_CollideMeshData.m_Faces, m_CollideMeshData.m_Edges);
}

//========================

TATRigidBody::TATRigidBody() :TATickable("RigidBody" + TString::ConvertInt(GetObjectIndex()))
{
	m_CollideShape = 0;
	Clear();
	m_IndexInPool = -1;
	m_FrictionCoefficient = 0.9;
	m_ContactHardness = 1;
	m_LinFactor.SetValue(1, 1, 1);
	m_AngFactor.SetValue(1, 1, 1);
}

void TATRigidBody::Clear()
{
	if (m_CollideShape)
		delete m_CollideShape;
	m_CollideShape = 0;
	m_BodyIndex = -1;
	m_DataIndex = -1;
	m_InertiaIndex = -1;
	m_InvMass = 0.0f;
	m_WorldTransform.SetIdentity();
}

bool TATRigidBody::Update(TATActor* actor, float dt)
{
	actor->m_WorldTransform = m_WorldTransform;
	if (actor->m_RenderUnit->m_UseTransform)
		actor->m_RenderUnit->m_Transform = m_WorldTransform;

	return true;
}

//@location in wcs
TATVector3 TATRigidBody::GetVelocityAtWCS(const TATVector3& location) const
{
	TATRigidBodyData& data = TATDynamicWorld::Instance()->m_RigidBodyDatas[m_DataIndex];

	TATVector3 r = location - GetMassCenter();

	return data.m_LinVel + data.m_AngVel.Cross(r);
}

//@location in lcs
TATVector3 TATRigidBody::GetVelocityAtLCS(const TATVector3& location) const
{
	return GetVelocityAtWCS(m_WorldTransform * location);
}

void TATRigidBody::ApplyImpulse(const TATVector3& impulse, const TATVector3& r)
{
	if (m_InvMass = 0)
		return;

	TATRigidBodyData& data = TATDynamicWorld::Instance()->m_RigidBodyDatas[m_DataIndex];
	data.m_LinVel += impulse * m_LinFactor * m_InvMass;

	TATVector3 torque = r.Cross(impulse * m_LinFactor);
	data.m_AngVel += TATDynamicWorld::Instance()->m_InertiaDatas[m_InertiaIndex].m_InvInertiaWorld * torque * m_AngFactor;
}

TATVector3 TATRigidBody::GetLinearVelocity() const
{
	return TATDynamicWorld::Instance()->m_RigidBodyDatas[m_DataIndex].m_LinVel;
}

TATVector3 TATRigidBody::GetAngularVelocity() const
{
	return TATDynamicWorld::Instance()->m_RigidBodyDatas[m_DataIndex].m_AngVel;
}


void TATRigidBody::UpdataInverseInertiaWorld() const
{
	TATMatrix3 m(m_WorldTransform.GetRotation());
	TATDynamicWorld::Instance()->m_InertiaDatas[m_InertiaIndex].m_InvInertiaWorld =
		m.Scaled(TATDynamicWorld::Instance()->m_InertiaDatas[m_InertiaIndex].m_InitInvInertia.GetDiagonal()) * m.Transpose();
}