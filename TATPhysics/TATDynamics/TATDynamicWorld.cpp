#include "TATDynamicWorld.h"
#include "../TATNarrowPhase/TATSAT.h"
#include "../TATNarrowPhase/TATCCD.h"
#include "TATPgsJacobiSolver.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"

class TATRigidBodyOverlapCallBack :public TATBvhCollideCallBack
{
public:
	TATRigidBodyOverlapCallBack(std::vector<TATBVCollisonPair>& collides) :m_Collides(collides)
	{}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2) override
	{
		m_Collides.push_back(TATBVCollisonPair(node1, node2));
	}

	std::vector<TATBVCollisonPair>& m_Collides;
};

TATDynamicWorld::TATDynamicWorld()
{
	m_RigidBodyBVTree.SetCollideSelf(true);
	m_ConstraintSolver = new TATPgsJacobiSolver;
}

void TATDynamicWorld::StepSimulation(float dt)
{
	m_RigidBodyBVTree.Clear();
	TATVector3 aabbMin, aabbMax;

	std::vector<TATBVCollisonPair> bpCollides;
	TATRigidBodyOverlapCallBack cb(bpCollides);
	for (int i = 0; i < m_RigidBodys.size(); i++)
	{
		TATRigidBody* rb = m_RigidBodys[i];
		rb->UpdateWorldAabb();
		rb->GetWorldAabb(aabbMin, aabbMax);

		TATBVNode* node = m_RigidBodyBVTree.InsertAabbNode(aabbMin, aabbMax);
		node->m_Data = (void*)(rb);
	}
	m_RigidBodyBVTree.FinishBuild();
	m_RigidBodyBVTree.CollideWithBVTree(&m_RigidBodyBVTree, &cb);

	TATRigidBody* rbA = 0;
	TATRigidBody* rbB = 0;
	std::vector<TATSATCollideData> satCollideDatas;
	for (int i = 0; i < bpCollides.size(); i++)
	{
		rbA = (TATRigidBody*)bpCollides[i].m_node1;
		rbB = (TATRigidBody*)bpCollides[i].m_node2;

		TATSATCollideData cd;
		if(TATSAT::SeparateAxisTest(TATRigidBodyGroup(rbA, rbB), cd))
			satCollideDatas.push_back(cd);
	}

	TATContactSolverInfo info;

	for (int i = 0; i < satCollideDatas.size(); i++)
	{
		m_ConstraintSolver->SolveContact(satCollideDatas[i], &m_RigidBodyDatas[0], &m_InertiaDatas[0], info);
	}

	m_ConstraintSolver->SolveFinish(&m_RigidBodyDatas[0], &m_InertiaDatas[0], info);

	for (int i = 0; i < m_RigidBodyDatas.size(); i++)
	{
		TATransformUtil::IntegrateTransform(&m_RigidBodyDatas[i], info.m_TimeStep, info.m_Damping, TATVector3(0, -9, 0));
	}
}