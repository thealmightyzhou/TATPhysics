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

TATDynamicWorld::TATDynamicWorld():m_RigidBodys(TAT_MAXRIGIDBODY_COUNT),
								   m_RigidBodyDatas(TAT_MAXRIGIDBODY_COUNT),
								   m_InertiaDatas(TAT_MAXRIGIDBODY_COUNT)
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

	const std::vector<TATRigidBody*> rigidbodys = m_RigidBodys.FetchAllUsed();
	for (int i = 0; i < (int)rigidbodys.size(); i++)
	{
		TATRigidBody* rb = rigidbodys[i];
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
	for (int i = 0; i < (int)bpCollides.size(); i++)
	{
		rbA = (TATRigidBody*)bpCollides[i].m_node1;
		rbB = (TATRigidBody*)bpCollides[i].m_node2;

		TATSATCollideData cd;
		if(TATSAT::SeparateAxisTest(TATRigidBodyGroup(rbA, rbB), cd))
			satCollideDatas.push_back(cd);
	}

	TATContactSolverInfo info;

	for (int i = 0; i < (int)satCollideDatas.size(); i++)
	{
		m_ConstraintSolver->SolveContact(satCollideDatas[i], m_RigidBodyDatas.GetPool(), m_InertiaDatas.GetPool(), info);
	}

	m_ConstraintSolver->SolveFinish(m_RigidBodyDatas.GetPool(), m_InertiaDatas.GetPool(), info);

	const std::vector<TATRigidBodyData*> rbDatas = m_RigidBodyDatas.FetchAllUsed();
	for (int i = 0; i < rbDatas.size(); i++)
	{
		TATransformUtil::IntegrateTransform(rbDatas[i], info.m_TimeStep, info.m_Damping, TATVector3(0, -9, 0));
	}
}

TATRigidBody* TATDynamicWorld::CreateConvex(TATMesh* mesh)
{
	TATPhyMeshData data;
	data.m_Vertices.resize(mesh->m_VertexCount);
	for (int i = 0; i < mesh->m_VertexCount; ++i)
	{
		data.m_Vertices[i].m_Position = mesh->m_MeshVertices[i].m_Position;
	}
	data.m_Faces.resize(mesh->m_FaceCount);
	for (int i = 0; i < mesh->m_FaceCount; ++i)
	{
		data.m_Faces[i].m_VertexIndices[0] = mesh->m_MeshFaces[i].m_Vertices[0];
		data.m_Faces[i].m_VertexIndices[1] = mesh->m_MeshFaces[i].m_Vertices[1];
		data.m_Faces[i].m_VertexIndices[2] = mesh->m_MeshFaces[i].m_Vertices[2];
	}

	TATCollideShapeConvex* convex = new TATCollideShapeConvex(data);

	TATRigidBodyData* rbdata = m_RigidBodyDatas.FetchUnused();
	TATInertiaData* in =  m_InertiaDatas.FetchUnused();
	in->m_InitInvInertia = convex->m_LocalInvInertiaTensor;

	TATRigidBody* rb = m_RigidBodys.FetchUnused();
	rb->SetCollideShape(convex);

	TATSolverBody* body = m_ConstraintSolver->m_SolverBodyPool.FetchUnused();

	body->m_OriginalBodyIndex = rb->m_IndexInPool;
	rb->m_BodyIndex = body->m_IndexInPool;
	rb->m_DataIndex = rbdata->m_IndexInPool;
	rb->m_InertiaIndex = in->m_IndexInPool;

}

void TATDynamicWorld::DestroyRigidBody(TATRigidBody* rb)
{
	m_InertiaDatas.ReturnUsed(rb->m_InertiaIndex);
	m_RigidBodyDatas.ReturnUsed(rb->m_DataIndex);
	m_RigidBodys.ReturnUsed(rb->m_IndexInPool);
}