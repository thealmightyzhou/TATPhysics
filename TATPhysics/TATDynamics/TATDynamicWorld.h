#pragma once
#include <map>
#include "TATRigidBody.h"
#include "TATRigidBodyData.h"
#include "../TATBroadPhase/TATBvh.h"
#include "../TATCommon/TATSingleton.h"
#include "../TATResources/TATMesh.h"

class TATPgsJacobiSolver;

class TATDynamicWorld:public Singleton<TATDynamicWorld>
{
public:
	TATDynamicWorld();

	TATObjectPool<TATRigidBody> m_RigidBodys;

	TATObjectPool<TATRigidBodyData> m_RigidBodyDatas;

	TATObjectPool<TATInertiaData> m_InertiaDatas;

	TATPgsJacobiSolver* m_ConstraintSolver;

	TATBvh m_RigidBodyBVTree;

	int m_BodyCount;

	TATVector3 m_GlobalGravity;

	void StepSimulation(float dt);

	void DestroyRigidBody(TATRigidBody* rb);

	TATRigidBody* CreateConvex(TATMesh* mesh, float invMass);

	void InitializeRb(TATRigidBody* rb, const TATransform& tr, float invMass, float restituitionCoeff, float frictionCoeff);

	void SyncRigidBodyData(TATRigidBody* rb);
};