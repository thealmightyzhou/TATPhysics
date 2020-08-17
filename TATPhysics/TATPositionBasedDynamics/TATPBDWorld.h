#pragma once

#include "../TATCommon/TATCore.h"
#include "../TATCommon/TATSingleton.h"
#include "../TATGeometry/TATMeshInfo.h"
#include "../TATBasis/TString.h"
#include <map>

class TATPBDBody;
class TATPBDConstraint;
class TATRigidBody;
class TATPBDParticle;

class TATSoftRigidCollideListener
{
public:
	virtual void GenerateCollision() {}
};

struct TATSoftRigidCollideData
{
	TATRigidBody* m_Rigid;
	TATPBDParticle* m_Particle;
	float m_Penetration;
	TATVector3 m_CollideNormal; // point form rigid to soft
	TATVector3 m_SoftPt;
	TATVector3 m_RigidPt;
};

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

	virtual void ProcessCollision();

	void ProjectCollision(const TATSoftRigidCollideData& data);

	void AddCollideProcessor(TATSoftRigidCollideListener* processor)
	{
		m_SoftRigidCollisions.push_back(processor);
	}

	void AddSoftRigidCollideData(const TATSoftRigidCollideData& data)
	{
		m_SoftRigidCollideDatas.push_back(data);
	}

	int m_IterateNum;

protected:
	std::map<int, TATPBDConstraint*> m_Constraints;

	std::map<TString, TATPBDBody*> m_PhyBodys;

	std::vector<TATSoftRigidCollideListener*> m_SoftRigidCollisions;

	std::vector<TATSoftRigidCollideData> m_SoftRigidCollideDatas;
};