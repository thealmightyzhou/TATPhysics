#pragma once
#include <map>
#include "TATRigidBody.h"
#include "../TATBroadPhase/TATBvh.h"

class TATPgsJacobiSolver;

class TATDynamicWorld
{
public:
	TATDynamicWorld()
	{
		m_RigidBodyBVTree.SetCollideSelf(true);
		m_ConstraintSolver = new TATPgsJacobiSolver;
	}

	std::vector<TATRigidBody*> m_RigidBodys;

	std::vector<TATRigidBodyData> m_RigidBodyDatas;
	//TATRigidBodyData* m_RigidBodyDatas;

	std::vector<TATInertiaData> m_InertiaDatas;
	//TATInertiaData* m_InertiaDatas;

	TATPgsJacobiSolver* m_ConstraintSolver;

	TATBvh m_RigidBodyBVTree;

	int m_BodyCount;

	void StepSimulation(float dt);

	void AddRigidBody();
};