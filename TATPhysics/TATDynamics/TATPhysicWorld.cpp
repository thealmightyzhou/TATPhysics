#include "TATPhysicWorld.h"
#include "TATDynamicWorld.h"
#include "../TATPositionBasedDynamics/TATPBDWorld.h"
#include "../TATBasis/TATErrorReporter.h"

TATDynamicWorld* TATPhysicWorld::m_DynWorld = 0;
TATPBDWorld* TATPhysicWorld::m_PBDWorld = 0;

void TATPhysicWorld::SetWorld(TATDynamicWorld* dyn, TATPBDWorld* softworld)
{
	m_DynWorld = dyn;
	m_PBDWorld = softworld;
	m_bInitialized = true;
}

void TATPhysicWorld::StepSimulation(float dt)
{
	if (!m_bInitialized)
	{
		TATErrorReporter::Instance()->ReportErr("physic world hasn't been initialized!");
		return;
	}

	SimulationBegin(dt);

	PrepareSolveConstraint(dt);

	for (int i = 0; i < m_IterationNum; ++i)
	{
		SolveConstraint(dt);
	}

	SimulationEnd(dt);

	Integrate(dt);
}

void TATPhysicWorld::SimulationBegin(float dt)
{

}

void TATPhysicWorld::PrepareSolveConstraint(float dt)
{

}

void TATPhysicWorld::SolveConstraint(float dt)
{

}

void TATPhysicWorld::SimulationEnd(float dt)
{

}

void TATPhysicWorld::Integrate(float dt)
{

}