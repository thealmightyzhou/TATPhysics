#include "TATInertia.h"
#include "../TATDynamics/TATRigidBody.h"

TATMatrix3 TATInertiaComputer::ComputeInertia(TATCollideShapePrimitive* cShape)
{
	CollideShapeType type = cShape->GetShapeType();
	TATCollideShapeSphere* sphere = 0;
	TATCollideShapeCuboid* cb = 0;
	TATCollideShapeConvex* cx = 0;
	TATCollideShapePlane* cp = 0;
	switch (type)
	{
	case CollideShapeType::CollideSphere:
		sphere = dynamic_cast<TATCollideShapeSphere*>(cShape);
		return SphereInertia(sphere->m_Center, sphere->m_Radius, sphere->m_InvMass);
		break;
	case CollideShapeType::CollideCuboid:
		cb = dynamic_cast<TATCollideShapeCuboid*>(cShape);
		return CuboidInertia(cb->m_Center, cb->m_Extend, cb->m_InvMass);
		break;
	case CollideShapeType::CollideConvex:
		cx = dynamic_cast<TATCollideShapeConvex*>(cShape);
		return ConvexInertia(cx->m_CollideMeshData, cx->m_LocalMassCenter, cx->m_InvMass);
		break;
	case CollideShapeType::CollidePlane:
		cp = dynamic_cast<TATCollideShapePlane*>(cShape);
		return TATMatrix3::GetIdentity();
	default:
		break;
	}

	return TATMatrix3::GetIdentity();
}

TATMatrix3 TATInertiaComputer::ConvexInertia(const TATPhyMeshData& meshData, const TATVector3& massct, float im)
{
	return TATPhyMeshDataComputer::ComputeInertiaTensor(meshData.m_Vertices, massct, float(1.0) / im);
}