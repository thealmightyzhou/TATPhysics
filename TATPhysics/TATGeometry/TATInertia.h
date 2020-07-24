#pragma once

#include "TATMeshInfo.h"
#include "../TATDynamics/TATRigidBody.h"

class TATCollideShapePrimitive;

class TATInertiaComputer
{
public:
	static TATMatrix3 ComputeInertia(TATCollideShapePrimitive* cShape);

	static TATMatrix3 SphereInertia(const TATVector3& c, float r, float im)
	{
		return TATMatrix3::GetIdentity(); //TODO
	}

	static TATMatrix3 CuboidInertia(const TATVector3& center, const TATVector3& extend, float im)
	{
		return TATMatrix3::GetIdentity(); //TODO
	}

	static TATMatrix3 ConvexInertia(const TATPhyMeshData& meshData, const TATVector3& massct, float im);

	//TODO
};