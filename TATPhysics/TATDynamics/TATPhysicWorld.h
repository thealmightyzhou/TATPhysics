#pragma once

#include "../TATBroadPhase/TATBvh.h"
#include "../TATCommon/TATSingleton.h"

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

	void SetWorld(TATDynamicWorld* dyn, TATPBDWorld* softworld);

	//to generate collision between objs not include self collision
	TATBvh m_PhysicObjectBVH;

	int m_IterationNum;

protected:

	TATDynamicWorld* m_DynWorld;

	TATPBDWorld* m_PBDWorld;

	bool m_bInitialized;
};