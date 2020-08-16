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
			TATPBDVertexDistConstraint* constr = dynamic_cast<TATPBDVertexDistConstraint*>(it->second);
			if (constr)
			{
				if (constr->m_Vertex0->m_Index == 0 || constr->m_Vertex1->m_Index == 0)
				{
					int stop = 1;
				}
			}

			it->second->SolveConstraint();
			it++;
		}
	}

	ite = m_PhyBodys.begin();
	while (ite != m_PhyBodys.end())
	{
		ite->second->Integrate(dt);
		ite++;
	}
}