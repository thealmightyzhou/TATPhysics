#include "TATPBDWorld.h"
#include "TATPBDConstraint.h"
#include "TATPBDBody.h"
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATNarrowPhase/TATSoftRigidCollisionEntry.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"

bool TATPBDWorld::AddBody(const TString& name, TATPBDBody* body)
{
	if (m_PhyBodys.find(name) == m_PhyBodys.end())
	{
		m_PhyBodys.insert(std::make_pair(name, body));
		return true;
	}

	else
		return false;
}

bool TATPBDWorld::AddConstraint(TATPBDConstraint* constr)
{
	if (m_Constraints.find(constr->GetHashValue()) == m_Constraints.end())
	{
		m_Constraints.insert(std::make_pair(constr->GetHashValue(), constr));
		return true;
	}
	return false;
}

void TATPBDWorld::RemoveConstraint(int hash)
{
	m_Constraints.erase(hash);
}

TATPBDConstraint* TATPBDWorld::GetConstraint(int hash)
{
	if (m_Constraints.find(hash) != m_Constraints.end())
		return m_Constraints[hash];
	else
		return 0;
}

void TATPBDWorld::StepSimulation(float dt)
{
	std::map<TString, TATPBDBody*>::iterator ite = m_PhyBodys.begin();
	while (ite != m_PhyBodys.end())
	{
		ite->second->StepSimulation(dt);
		ite++;
	}

	std::map<int, TATPBDConstraint*>::iterator it;
	for (int i = 0; i < m_IterateNum; ++i)
	{
		it = m_Constraints.begin();
		while (it != m_Constraints.end())
		{
			it->second->SolveConstraint();
			it++;
		}
	}

	m_PhyBodyBVH.Clear();
	ite = m_PhyBodys.begin();
	TATVector3 min, max;
	while (ite != m_PhyBodys.end())
	{
		ite->second->UpdateAabb();
		ite->second->m_ParticleBVH.GetBound(min, max);
		TATBVNode* node = m_PhyBodyBVH.InsertAabbNode(min, max);
		node->m_Data = (void*)(ite->second);
		ite++;
	}
	m_PhyBodyBVH.FinishBuild();

	ProcessCollision(dt);

	ite = m_PhyBodys.begin();
	while (ite != m_PhyBodys.end())
	{
		ite->second->Integrate(dt);
		ite++;
	}
}

class TATSoftRigidCollisionProcessor:public TATBvhCollideCallBack
{
public:
	TATSoftRigidCollisionProcessor(std::vector<TATSoftRigidCollideData>& collidedatas, float dt) :
		m_CollideDatas(collidedatas), m_DeltaTime(dt)
	{}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
	{
		TATPBDBody* soft = (TATPBDBody*)node1->m_Data;
		TATRigidBody* rigid = (TATRigidBody*)(node2->m_Data);

		std::vector<TATSoftRigidCollideData> datas;
		if (TATSoftRigidCollisionEntry::ProcessSoftRigidCollision(soft, rigid, m_DeltaTime, datas))
		{
			m_CollideDatas.insert(m_CollideDatas.end(), datas.begin(), datas.end());
		}
	}

	std::vector<TATSoftRigidCollideData>& m_CollideDatas;
	float m_DeltaTime;
};

void TATPBDWorld::ProcessCollision(float dt)
{
	TATSoftRigidCollisionProcessor cb(m_SoftRigidCollideDatas, dt);
	TATDynamicWorld::Instance()->m_RigidBodyBVTree.CollideWithBVTree(&m_PhyBodyBVH, &cb); //TODO refactory

	for (int i = 0; i < m_SoftRigidCollideDatas.size(); ++i)
	{
		ProjectRSCollision(m_SoftRigidCollideDatas[i], dt);
	}

	m_SoftRigidCollideDatas.clear();
}

void TATPBDWorld::ProjectRSCollision(const TATSoftRigidCollideData& data, float dt)
{
	TATPBDParticle* particle = data.m_Particle;
	TATRigidBody* rigid = data.m_Rigid;
	const TATMatrix3& iwi = TATDynamicWorld::Instance()->m_InertiaDatas[rigid->m_InertiaIndex].m_InvInertiaWorld;
	//TATVector3 r = data.m_RigidPt - rigid->GetMassCenter();
	TATVector3 r = data.m_SoftPt - rigid->GetMassCenter();

	TATMatrix3 impulseMatrix = TATCollisionUtil::ImpulseMatrix
	(
		dt,
		data.m_Particle->m_InvMass,
		rigid->m_InvMass,
		iwi, 
		r
	);

	//TATVector3 va = rigid->GetVelocityAtWCS(data.m_RigidPt) * dt;
	TATVector3 va = rigid->GetVelocityAtWCS(data.m_SoftPt) * dt;
	TATVector3 vb = particle->m_PredictPos - particle->Position();
	const TATVector3 rel_vel = vb - va;
	const float rel_vel_normal = rel_vel.Dot(data.m_CollideNormal);
	const TATVector3 rel_frict_vel = rel_vel - rel_vel_normal * data.m_CollideNormal;
	float frict = particle->m_HostBody->m_FrictionCoeffcient * rigid->m_FrictionCoefficient;
	float fricCoeff = rel_frict_vel.Length() < rel_vel_normal * particle->m_HostBody->m_FrictionCoeffcient ? 0 : 1 - frict;

	float kst = 1.0f;

	const TATVector3 impulse = impulseMatrix *
		(rel_vel - rel_frict_vel * fricCoeff - data.m_Penetration * rigid->m_ContactHardness * data.m_CollideNormal) * kst;

	particle->m_PredictPos -= impulse * particle->m_InvMass * dt;
	//particle->m_Velocity -= impulse * particle->m_InvMass;

	rigid->ApplyImpulse(impulse, r);

	//TATDynamicWorld::Instance()->ExtraIntegrate(rigid, dt);
	particle->m_PredictPos = data.m_SoftPt + data.m_CollideNormal * 0.04;
	particle->m_Velocity -= impulse * particle->m_InvMass;

}