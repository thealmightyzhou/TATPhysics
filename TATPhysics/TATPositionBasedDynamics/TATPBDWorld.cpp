#include "TATPBDWorld.h"
#include "TATPBDConstraint.h"
#include "TATPBDBody.h"

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

	ite = m_PhyBodys.begin();
	while (ite != m_PhyBodys.end())
	{
		ite->second->UpdateAabb();
		ite++;
	}

	ProcessCollision();

	ite = m_PhyBodys.begin();
	while (ite != m_PhyBodys.end())
	{
		ite->second->Integrate(dt);
		ite++;
	}
}

void TATPBDWorld::ProcessCollision()
{
	for (int i = 0; i < m_SoftRigidCollisions.size(); ++i)
	{
		m_SoftRigidCollisions[i]->GenerateCollision();
	}

	for (int i = 0; i < m_SoftRigidCollideDatas.size(); ++i)
	{
		ProjectCollision(m_SoftRigidCollideDatas[i]);
	}

	m_SoftRigidCollideDatas.clear();
}

void TATPBDWorld::ProjectCollision(const TATSoftRigidCollideData& data)
{
	data.m_Particle->m_PredictPos = data.m_SoftPt;
}