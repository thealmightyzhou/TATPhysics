#pragma once
#include <map>
#include "TATRigidBody.h"
#include "TATRigidBodyData.h"
#include "../TATBroadPhase/TATBvh.h"
#include "../TATCommon/TATSingleton.h"
#include "../TATResources/TATMesh.h"
#include "TATSolverConstraint.h"
#include "../TATNarrowPhase/TATRigidBodyCollideData.h"

class TATPgsJacobiSolver;


class TATDynamicWorld:public Singleton<TATDynamicWorld>
{
public:
	TATDynamicWorld();

	TATObjectPool<TATRigidBody> m_RigidBodys;

	//TATObjectPool<TATRigidBodyData> m_RigidBodyDatas;

	//TATObjectPool<TATInertiaData> m_InertiaDatas;

	TATPgsJacobiSolver* m_ConstraintSolver;

	TATBvh m_RigidBodyBVTree;

	int m_BodyCount;

	TATVector3 m_GlobalGravity;

	void StepSimulation(float dt);

	void SimulationBegin(float dt);

	void PrepareSolve(std::vector<TATRigidBodyCollideData>& contacts, float dt);

	void SolveConstraint(float dt);

	void Integrate(float dt);

	void SimulationEnd(float dt);

	void DestroyRigidBody(TATRigidBody* rb);

	TATRigidBody* CreateConvex(TATMesh* mesh, float invMass);

	//static plane
	TATRigidBody* CreatePlane(const TATVector3& origin, const TATVector3& normal);

	TATRigidBody* CreateSphere(const TATVector3& ct, float radius, float invMass);

	void InitRigidBody(TATRigidBody* rb, const TATransform& tr, float invMass, float restituitionCoeff, float frictionCoeff, const TATVector3& g);

	void SyncRigidBodyData(TATRigidBody* rb);

	TATContactSolverInfo m_GlobalInfo;
};