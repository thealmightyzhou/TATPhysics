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

		min = particles[0].m_CurrPos;
		max = particles[0].m_CurrPos;
		for (int p = 0; p < particles.size(); ++p)
		{
			if (soft[i]->m_UseCCD)
			{
				min.SetMin(particles[p].m_CurrPos);
				min.SetMin(particles[p].m_LastPos);
				max.SetMax(particles[p].m_CurrPos);
				max.SetMax(particles[p].m_LastPos);
			}
			else
			{
				min.SetMin(particles[p].m_CurrPos);
				max.SetMax(particles[p].m_CurrPos);
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
	PhysicObjectOverlapCB(float dt, std::vector<TATRigidBodyCollideData>& rr, std::vector<TATSoftRigidCollideData>& rs, std::vector<TATSoftSoftCollideData>& ss) :
		m_TimeStep(dt), m_RRCollides(rr), m_RSCollides(rs), m_SSCollides(ss)
	{

	}
	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2) 
	{
		TATRigidBody* rb1 = (TATRigidBody*)(node1->m_Data);
		TATRigidBody* rb2 = (TATRigidBody*)(node2->m_Data);
		TATPBDBody* soft1 = (TATPBDBody*)(node1->m_Data);
		TATPBDBody* soft2 = (TATPBDBody*)(node2->m_Data);

		if (rb1->Legal() && rb2->Legal() && rb1->m_Type == TATPhyBody::BodyType::RigidBody && rb2->m_Type == TATPhyBody::BodyType::RigidBody)
		{
			TATRigidBodyCollideData data;
			if (TATRigidBodyCollisionEntry::ProcessRigidCollision(rb1, rb2, data))
			{
				data.m_RigidA = rb1;
				data.m_RigidB = rb2;
				m_RRCollides.push_back(data);
			}
		}
		else if (rb1->Legal() && soft2->Legal() && rb1->m_Type == TATPhyBody::BodyType::RigidBody && soft2->m_Type == TATPhyBody::BodyType::SoftBody)
		{
			std::vector<TATSoftRigidCollideData> datas;
			if (TATSoftRigidCollisionEntry::ProcessSoftRigidCollision(soft2, rb1, m_TimeStep, datas))
			{
				m_RSCollides.insert(m_RSCollides.end(), datas.begin(), datas.end());
			}
		}
		else if (soft1->Legal() && rb2->Legal() && soft1->m_Type == TATPhyBody::BodyType::SoftBody && rb2->m_Type == TATPhyBody::BodyType::RigidBody)
		{
			std::vector<TATSoftRigidCollideData> datas;
			if (TATSoftRigidCollisionEntry::ProcessSoftRigidCollision(soft1, rb2, m_TimeStep, datas))
			{
				m_RSCollides.insert(m_RSCollides.end(), datas.begin(), datas.end());
			}
		}
		else if (soft1->Legal() && soft2->Legal() && soft1->m_Type == TATPhyBody::BodyType::SoftBody && soft2->m_Type == TATPhyBody::BodyType::SoftBody)
		{
			std::vector<TATSoftSoftCollideData> datas;
			//TODO
		}
	}

	float m_TimeStep;
	std::vector<TATRigidBodyCollideData>& m_RRCollides;
	std::vector<TATSoftRigidCollideData>& m_RSCollides;
	std::vector<TATSoftSoftCollideData>& m_SSCollides;
};

void TATPhysicWorld::PrepareSolveConstraint(float dt)
{
	PhysicObjectOverlapCB cb(dt, m_RRCollides, m_RSCollides, m_SSCollides);
	m_PhysicObjectBVH.CollideWithBVTree(&m_PhysicObjectBVH, &cb);

	m_DynWorld->PrepareSolve(m_RRCollides, dt);

	m_PBDWorld->PrepareSolve(m_SSCollides, dt);
}

void TATPhysicWorld::SolveConstraint(float dt)
{
	m_DynWorld->SolveConstraint(dt);

	m_PBDWorld->SolveConstraint(dt);

	SolveRSContacts(dt);
}

void TATPhysicWorld::SolveRSContacts(float dt)
{
	TATSoftRigidCollisionEntry::SolveRSContacts(m_RSCollides, dt);
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

	m_RRCollides.clear();
	m_RSCollides.clear();
	m_SSCollides.clear();
}

