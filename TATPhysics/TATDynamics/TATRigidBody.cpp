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

TATCollideShapeConvex::TATCollideShapeConvex(const TATPhyMeshData& meshData) :m_CollideMeshData(meshData)
{
	TATVector3 min, max;
	m_LocalMassCenter = TATPhyMeshDataComputer::ComputeMassCentreAndLocalAabb(m_CollideMeshData.m_Vertices, min, max);
	m_LocalAabb.SetOrigin(min, max);
	m_LocalInertiaTensor = TATInertiaComputer::ComputeInertia(this);
	m_LocalInvInertiaTensor = m_LocalInertiaTensor.Inverse();
}