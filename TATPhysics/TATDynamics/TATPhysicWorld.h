#pragma once

#include "../TATBroadPhase/TATBvh.h"
#include "../TATCommon/TATSingleton.h"
#include "../TATNarrowPhase/TATRigidBodyCollideData.h"
#include "../TATNarrowPhase/TATSoftRigidCollisionData.h"
#include "../TATNarrowPhase/TATSoftSoftCollisionData.h"

class TATDynamicWorld;
class TATPBDWorld;

class TATPhysicWorld : public Singleton<TATPhysicWorld>
{
public:
	TATPhysicWorld()
	{
		m_IterationNum = 4;
		m_bInitialized = false;
	}

	virtual void StepSimulation(float dt);

	virtual void SimulationBegin(float dt);

	virtual void PrepareSolveConstraint(float dt);

	virtual void SolveConstraint(float dt);

	virtual void SimulationEnd(float dt);

	virtual void Integrate(float dt);

	virtual void SolveRSContacts(float dt);

	void SetWorld(TATDynamicWorld* dyn, TATPBDWorld* softworld);

	//to generate collision between objs not include self collision
	TATBvh m_PhysicObjectBVH;

	int m_IterationNum;

protected:

	TATDynamicWorld* m_DynWorld;

	TATPBDWorld* m_PBDWorld;

	bool m_bInitialized;

	std::vector<TATRigidBodyCollideData> m_RRCollides;
	std::vector<TATSoftRigidCollideData> m_RSCollides;
	std::vector<TATSoftSoftCollideData> m_SSCollides;
};