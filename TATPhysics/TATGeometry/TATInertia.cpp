#include "TATInertia.h"
#include "../TATDynamics/TATRigidBody.h"

TATMatrix3 TATInertiaComputer::ComputeInertia(TATCollideShapePrimitive* cShape)
{
	CollideShapeType type = cShape->GetShapeType();
	switch (type)
	{
	case CollideShapeType::CollideSphere:
		TATCollideShapeSphere* sphere = dynamic_cast<TATCollideShapeSphere*>(cShape);
		return SphereInertia(sphere->m_Center, sphere->m_Radius, sphere->m_InvMass);
		break;
	case CollideShapeType::CollideCuboid:
		TATCollideShapeCuboid* cb = dynamic_cast<TATCollideShapeCuboid*>(cShape);
		return CuboidInertia(cb->m_Center, cb->m_Extend, cb->m_InvMass);
		break;
	case CollideShapeType::CollideConvex:
		TATCollideShapeConvex* cx = dynamic_cast<TATCollideShapeConvex*>(cShape);
		return ConvexInertia(cx->m_CollideMeshData, cx->m_LocalMassCenter, cx->m_InvMass);
	default:
		break;
	}
}

TATMatrix3 TATInertiaComputer::ConvexInertia(const TATPhyMeshData& meshData, const TATVector3& massct, float im)
{
	return TATPhyMeshDataComputer::ComputeInertiaTensor(meshData.m_Vertices, massct, float(1.0) / im);
}