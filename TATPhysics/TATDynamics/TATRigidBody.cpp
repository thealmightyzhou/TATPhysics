#include "TATRigidBody.h"
#include "../TATGeometry/TATInertia.h"

TATCollideShapeSphere::TATCollideShapeSphere(const TATVector3& center, float radius) :m_Center(center), m_Radius(radius)
{
	m_LocalMassCenter = m_Center;
	m_LocalInertiaTensor = TATInertiaComputer::ComputeInertia(this);
	m_LocalInvInertiaTensor = m_LocalInertiaTensor.Inverse();
	m_LocalAabb.SetOriginSphere(center, radius);
}

//=======================

TATCollideShapeCuboid::TATCollideShapeCuboid(const TATVector3& center, const TATVector3& extend) :m_Center(center), m_Extend(extend)
{
	m_LocalMassCenter = m_Center;
	m_LocalInertiaTensor = TATInertiaComputer::ComputeInertia(this);
	m_LocalInvInertiaTensor = m_LocalInertiaTensor.Inverse();
	m_LocalAabb.SetOriginExtend(center, extend);
}

//========================
//will automatically fill the physic infomation
TATCollideShapeConvex::TATCollideShapeConvex(const TATPhyMeshData& meshData) :m_CollideMeshData(meshData)
{
	TATVector3 min, max;
	m_LocalMassCenter = TATPhyMeshDataComputer::ComputeMassCentreAndLocalAabb(m_CollideMeshData.m_Vertices, min, max);
	m_LocalAabb.SetOrigin(min, max);
	m_LocalInertiaTensor = TATInertiaComputer::ComputeInertia(this);
	m_LocalInvInertiaTensor = m_LocalInertiaTensor.Inverse();
	TATPhyMeshDataComputer::CompleteMeshData(m_CollideMeshData.m_Vertices, m_CollideMeshData.m_Faces, m_CollideMeshData.m_Edges);
}

TATRigidBody::TATRigidBody()
{
	Clear();
	m_IndexInPool = -1;
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
