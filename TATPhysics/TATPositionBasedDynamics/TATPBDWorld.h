#pragma once

#include "../TATCommon/TATCore.h"
#include "../TATCommon/TATSingleton.h"
#include "../TATGeometry/TATMeshInfo.h"
#include "../TATBasis/TString.h"
#include "../TATNarrowPhase/TATSoftRigidCollisionData.h"
#include "../TATBroadPhase/TATBvh.h"
#include <map>

class TATPBDBody;
class TATPBDConstraint;
class TATRigidBody;
class TATPBDParticle;

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

	virtual void ProcessCollision(float dt);

	void ProjectCollision(const TATSoftRigidCollideData& data);

	void AddSoftRigidCollideData(const TATSoftRigidCollideData& data)
	{
		m_SoftRigidCollideDatas.push_back(data);
	}

	void AddSoftRigidCollideData(const std::vector<TATSoftRigidCollideData>& datas)
	{
		m_SoftRigidCollideDatas.insert(m_SoftRigidCollideDatas.end(), datas.begin(), datas.end());
	}

	int m_IterateNum;

protected:
	std::map<int, TATPBDConstraint*> m_Constraints;

	std::map<TString, TATPBDBody*> m_PhyBodys;

	std::vector<TATSoftRigidCollideData> m_SoftRigidCollideDatas;

	TATBvh m_PhyBodyBVH;
};