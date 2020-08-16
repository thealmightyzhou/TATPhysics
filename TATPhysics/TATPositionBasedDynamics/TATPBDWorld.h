#pragma once

#include "../TATCommon/TATCore.h"
#include "../TATCommon/TATSingleton.h"
#include "../TATGeometry/TATMeshInfo.h"
#include "../TATBasis/TString.h"
#include <map>

class TATPBDBody;
class TATPBDConstraint;

//position based dynamics entry
class TATPBDWorld:public Singleton<TATPBDWorld>
{
public:
	TATPBDWorld()
	{
		m_IterateNum = 4;
	}

	bool AddBody(const TString& name, TATPBDBody* body);

	bool AddConstraint(TATPBDConstraint* constr);

	void RemoveConstraint(int hash);

	TATPBDConstraint* GetConstraint(int hash);

	void StepSimulation(float dt);

	int m_IterateNum;

protected:
	std::map<int, TATPBDConstraint*> m_Constraints;

	std::map<TString, TATPBDBody*> m_PhyBodys;
};