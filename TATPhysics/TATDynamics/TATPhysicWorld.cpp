#include "TATPhysicWorld.h"
#include "TATDynamicWorld.h"
#include "../TATPositionBasedDynamics/TATPBDWorld.h"
#include "../TATPositionBasedDynamics/TATPBDBody.h"
#include "../TATBasis/TATErrorReporter.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"
#include "../TATNarrowPhase/TATRigidBodyCollisionEntry.h"
#include "../TATNarrowPhase/TATSoftRigidCollisionEntry.h"


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

	Integrate(dt);

	SimulationEnd(dt);
}

void TATPhysicWorld::SimulationBegin(float dt)
{
	m_PhysicObjectBVH.Clear();

	TATVector3 min, max;
	TATVector3 min1, max1;

	const std::vector<TATRigidBody*>& rigidbodys = m_DynWorld->m_RigidBodys.FetchAllUsed();

	for (int i = 0; i < rigidbodys.size(); ++i)
	{
		TATRigidBody* rb = rigidbodys[i];
		if (rb->m_UseCCD)
		{
			rb->m_CollideShape->m_LocalAabb.Update(rb->GetWorldTransform());
			rb->GetWorldAabb(min, max);
			rb->m_CollideShape->m_LocalAabb.Update(rb->m_PreWorldTransform);
			rb->GetWorldAabb(min1, max1);

			min.SetMin(min1);
			min.SetMin(max1);
			max.SetMax(min1);
			max.SetMax(max1);
		}
		else
		{
			rb->UpdateWorldAabb();
			rb->GetWorldAabb(min, max);
		}

		m_PhysicObjectBVH.InsertAabbNode(min, max)->m_Data = (void*)(rb);
	}

	//step particles first
	m_PBDWorld->SimulationBegin(dt);

	std::vector<TATPBDBody*> soft;
	m_PBDWorld->GetBodies(soft);
	for (int i = 0; i < soft.size(); ++i)
	{
		std::vector<TATPBDParticle>& particles = soft[i]->m_Particles;
		if (particles.size() == 0)
			continue;

		min = particles[0].m_PredictPos;
		max = particles[0].m_PredictPos;
		for (int p = 0; p < particles.size(); ++p)
		{
			if (soft[i]->m_UseCCD)
			{
				min.SetMin(particles[p].m_PredictPos);
				min.SetMin(particles[p].Position());
				max.SetMax(particles[p].m_PredictPos);
				max.SetMax(particles[p].Position());
			}
			else
			{
				min.SetMin(particles[p].m_PredictPos);
				max.SetMax(particles[p].m_PredictPos);
			}
		}
		
		m_PhysicObjectBVH.InsertAabbNode(min, max)->m_Data = (void*)(soft[i]);

	}

	m_DynWorld->SimulationBegin(dt);

	m_PhysicObjectBVH.FinishBuild();
	m_PhysicObjectBVH.SetCollideSelf(true);
}

class PhysicObjectOverlapCB :public TATBvhCollideCallBack
{
public:
	PhysicObjectOverlapCB(float dt):
		m_TimeStep(dt)
	{

	}
	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2) 
	{
		TATRigidBody* rb1 = (TATRigidBody*)(node1->m_Data);
		TATPBDBody* soft1 = (TATPBDBody*)(node1->m_Data);
		TATRigidBody* rb2 = (TATRigidBody*)(node2->m_Data);
		TATPBDBody* soft2 = (TATPBDBody*)(node2->m_Data);

		if (rb1 && rb2)
		{
			TATRigidBodyCollideData data;//TODO
			TATRigidBodyCollisionEntry::ProcessRigidCollision(rb1, rb2, data);
		}
		else if (rb1 && soft2)
		{
			std::vector<TATSoftRigidCollideData> datas;//TODO
			TATSoftRigidCollisionEntry::ProcessSoftRigidCollision(soft2, rb1, m_TimeStep, datas);
		}
		else if (soft1 && rb2)
		{
			std::vector<TATSoftRigidCollideData> datas;//TODO
			TATSoftRigidCollisionEntry::ProcessSoftRigidCollision(soft1, rb2, m_TimeStep, datas);
		}
		else if (soft1 && soft2)
		{
			//TODO
		}
	}

	float m_TimeStep;
};

void TATPhysicWorld::PrepareSolveConstraint(float dt)
{
	PhysicObjectOverlapCB cb(dt);
	m_PhysicObjectBVH.CollideWithBVTree(&m_PhysicObjectBVH, &cb);

	m_DynWorld->PrepareSolve(dt);

	m_PBDWorld->PrepareSolve(dt);
}

void TATPhysicWorld::SolveConstraint(float dt)
{
	m_DynWorld->SolveConstraint(dt);

	m_PBDWorld->SolveConstraint(dt);
}

void TATPhysicWorld::Integrate(float dt)
{
	m_DynWorld->Integrate(dt);

	m_PBDWorld->Integrate(dt);
}

void TATPhysicWorld::SimulationEnd(float dt)
{
	m_DynWorld->SimulationEnd(dt);

	m_PBDWorld->SimulationEnd(dt);
}

