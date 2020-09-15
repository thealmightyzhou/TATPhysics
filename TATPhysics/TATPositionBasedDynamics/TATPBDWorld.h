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

	void GetBodies(std::vector<TATPBDBody*>& bodies);

	void StepSimulation(float dt);

	void SimulationBegin(float dt);

	void PrepareSolve(float dt);

	void SolveConstraint(float dt);

	void Integrate(float dt);

	void SimulationEnd(float dt);

	virtual void ProcessCollision(float dt);

	void ProjectRSCollision(const TATSoftRigidCollideData& data, float dt);

	void AddSoftRigidCollideData(const TATSoftRigidCollideData& data)
	{
		m_SoftRigidCollideDatas.push_back(data);
	}

	void AddSoftRigidCollideData(const std::vector<TATSoftRigidCollideData>& datas)
	{
		m_SoftRigidCollideDatas.insert(m_SoftRigidCollideDatas.end(), datas.begin(), datas.end());
	}

	void ComputeMatrixK(const TATVector3& connector, float invMass, const TATVector3& x, TATMatrix3& iwi, TATMatrix3& K);

	void SolveCollision(const TATSoftRigidCollideData& data, float dt, float& sumImpulse, float stiff, float frictionStiff);

	void GenerateCollision(float dt);

	int m_IterateNum;

protected:
	std::map<int, TATPBDConstraint*> m_Constraints;

	std::map<TString, TATPBDBody*> m_PhyBodys;

	std::vector<TATSoftRigidCollideData> m_SoftRigidCollideDatas;

	TATBvh m_PhyBodyBVH;
};