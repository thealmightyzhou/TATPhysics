#include "TATDynamicWorld.h"
#include "../TATNarrowPhase/TATSAT.h"
#include "TATPgsJacobiSolver.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"
#include "../TATApplication/TATApplication.h"
#include "../TATApplication/TAThread.h"
#include "../TATNarrowPhase/TATRigidBodyCollisionEntry.h"


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

TATDynamicWorld::TATDynamicWorld():m_RigidBodys(TAT_MAXRIGIDBODY_COUNT)//,
								   //m_RigidBodyDatas(TAT_MAXRIGIDBODY_COUNT),
								   //m_InertiaDatas(TAT_MAXRIGIDBODY_COUNT)
{
	m_RigidBodyBVTree.SetCollideSelf(true);
	m_ConstraintSolver = new TATPgsJacobiSolver;
	m_GlobalGravity = TATVector3(0, -9, 0);
}

void TATDynamicWorld::StepSimulation(float dt)
{
	//broad phase
	//-----------------
	m_RigidBodyBVTree.Clear();
	TATVector3 aabbMin, aabbMax;

	std::vector<TATBVCollisonPair> bpCollides;
	TATRigidBodyOverlapCallBack cb(bpCollides);

	const std::vector<TATRigidBody*>& rigidbodys = m_RigidBodys.FetchAllUsed();
	for (int i = 0; i < (int)rigidbodys.size(); i++)
	{
		TATRigidBody* rb = rigidbodys[i];
		rb->m_PreWorldTransform = rb->m_WorldTransform;
		rb->UpdateWorldAabb();
		rb->GetWorldAabb(aabbMin, aabbMax);

		TATBVNode* node = m_RigidBodyBVTree.InsertAabbNode(aabbMin, aabbMax);
		node->m_Data = (void*)(rb);
	}
	m_RigidBodyBVTree.FinishBuild();
	m_RigidBodyBVTree.CollideWithBVTree(&m_RigidBodyBVTree, &cb);

	//narrow phase
	//-----------------

	TATRigidBody* rbA = 0;
	TATRigidBody* rbB = 0;
	std::vector<TATRigidBodyCollideData> rbCollideDatas;
	for (int i = 0; i < (int)bpCollides.size(); i++)
	{
		rbA = (TATRigidBody*)bpCollides[i].m_node1->m_Data;
		rbB = (TATRigidBody*)bpCollides[i].m_node2->m_Data;

		TATRigidBodyCollideData cd;
		if (TATRigidBodyCollisionEntry::ProcessRigidCollision(rbA, rbB, cd))
		{
			rbCollideDatas.push_back(cd);
		}
	}

	//solve
	//-----------------

	TATContactSolverInfo info;
	info.m_Damping = 0.98f;

	for (int i = 0; i < (int)rbCollideDatas.size(); i++)
	{
		TAT_RENDER_TASK_LIST->PushTask([](const TATVector3& p0, const TATVector3& p1, const TATVector3& col)->void
		{
			if (TAT_RENDER_THREAD->m_LinePainter)
			{
				TAT_RENDER_THREAD->m_LinePainter->Clear();
				TAT_RENDER_THREAD->m_LinePainter->PaintLine(p0, p1, col);
			}

		}, rbCollideDatas[i].m_CollidePt0, rbCollideDatas[i].m_CollidePt1, TATVector3(1, 0, 0));

		m_ConstraintSolver->SolveContact(rbCollideDatas[i], info);
	}

	//integrate
	//-----------------

	//const std::vector<TATRigidBodyData*> rbDatas = m_RigidBodyDatas.FetchAllUsed();
	//for (int i = 0; i < rbDatas.size(); i++)
	//{
	//	TATransformUtil::IntegrateTransform(rbDatas[i], info.m_TimeStep, info.m_Damping);
	//}

	for (int i = 0; i < rigidbodys.size(); i++)
	{
		TATransformUtil::IntegrateTransform(rigidbodys[i], info.m_TimeStep, info.m_Damping);
	}

	const std::vector<TATRigidBody*> rbs = m_RigidBodys.FetchAllUsed();
	for (int i = 0; i < rbs.size(); ++i)
	{
		SyncRigidBodyData(rbs[i]);
	}
}

void TATDynamicWorld::SimulationBegin(float dt)
{
	m_RigidBodyBVTree.Clear();
	TATVector3 aabbMin, aabbMax;

	const std::vector<TATRigidBody*>& rigidbodys = m_RigidBodys.FetchAllUsed();
	for (int i = 0; i < (int)rigidbodys.size(); i++)
	{
		TATRigidBody* rb = rigidbodys[i];
		rb->m_PreWorldTransform = rb->m_WorldTransform;
		rb->UpdateWorldAabb();
		rb->GetWorldAabb(aabbMin, aabbMax);

		TATBVNode* node = m_RigidBodyBVTree.InsertAabbNode(aabbMin, aabbMax);
		node->m_Data = (void*)(rb);
	}
	m_RigidBodyBVTree.FinishBuild();
}

void TATDynamicWorld::PrepareSolve(float dt)
{
	//broad phase
	//-----------------
	std::vector<TATBVCollisonPair> bpCollides;
	TATRigidBodyOverlapCallBack cb(bpCollides);
	m_RigidBodyBVTree.CollideWithBVTree(&m_RigidBodyBVTree, &cb);

	//narrow phase
	//-----------------
	std::vector<TATRigidBodyCollideData> contacts;
	TATRigidBody* rbA = 0;
	TATRigidBody* rbB = 0;
	for (int i = 0; i < (int)bpCollides.size(); i++)
	{
		rbA = (TATRigidBody*)bpCollides[i].m_node1->m_Data;
		rbB = (TATRigidBody*)bpCollides[i].m_node2->m_Data;

		TATRigidBodyCollideData cd;
		if (TATRigidBodyCollisionEntry::ProcessRigidCollision(rbA, rbB, cd))
		{
			contacts.push_back(cd);
		}
	}

	m_GlobalInfo.m_Damping = 0.98f;
	m_GlobalInfo.m_TimeStep = dt;

	m_ConstraintSolver->m_GlobalInfo = &m_GlobalInfo;
	m_ConstraintSolver->m_Contacts.assign(contacts.begin(), contacts.end());
	m_ConstraintSolver->PrepareSolve();
}

void TATDynamicWorld::SolveConstraint(float dt)
{
	m_ConstraintSolver->SolveConstraint();
}

void TATDynamicWorld::Integrate(float dt)
{
	m_ConstraintSolver->Integrate();

	const std::vector<TATRigidBody*>& rigidbodys = m_RigidBodys.FetchAllUsed();
	for (int i = 0; i < rigidbodys.size(); i++)
	{
		TATransformUtil::IntegrateTransform(rigidbodys[i], dt, m_GlobalInfo.m_Damping);
	}

	for (int i = 0; i < rigidbodys.size(); ++i)
	{
		SyncRigidBodyData(rigidbodys[i]);
	}
}

void TATDynamicWorld::SimulationEnd(float dt)
{
	m_ConstraintSolver->SimulationEnd();
}

TATRigidBody* TATDynamicWorld::CreateConvex(TATMesh* mesh, float invMass)
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

	TATCollideShapeConvex* convex = new TATCollideShapeConvex(data, invMass);

	//TATRigidBodyData* rbdata = m_RigidBodyDatas.FetchUnused();
	//TATInertiaData* in =  m_InertiaDatas.FetchUnused();

	TATRigidBody* rb = m_RigidBodys.FetchUnused();
	rb->SetCollideShape(convex);

	TATSolverBody* body = m_ConstraintSolver->m_SolverBodyPool.FetchUnused();

	body->m_OriginalBodyIndex = rb->m_IndexInPool;
	rb->m_BodyIndex = body->m_IndexInPool;
	//rb->m_DataIndex = rbdata->m_IndexInPool;
	//rb->m_InertiaIndex = in->m_IndexInPool;

	return rb;
}

TATRigidBody* TATDynamicWorld::CreatePlane(const TATVector3& origin, const TATVector3& normal)
{
	TATCollideShapePlane* plane = new TATCollideShapePlane(origin, normal);
	//TATRigidBodyData* rbdata = m_RigidBodyDatas.FetchUnused();
	//TATInertiaData* in = m_InertiaDatas.FetchUnused();
	TATRigidBody* rb = m_RigidBodys.FetchUnused();
	rb->SetCollideShape(plane);

	//rb->m_DataIndex = rbdata->m_IndexInPool;
	//rb->m_InertiaIndex = in->m_IndexInPool;

	rb->m_InvMass = 0;
	rb->m_InitInvInertia = rb->m_CollideShape->m_LocalInvInertiaTensor;
	rb->m_InvInertiaWorld = rb->m_InitInvInertia;
	//rbdata->m_InvMass = 0;

	//in->m_InitInvInertia = rb->m_CollideShape->m_LocalInvInertiaTensor;
	//in->m_InvInertiaWorld = in->m_InitInvInertia;

	return rb;
}

TATRigidBody* TATDynamicWorld::CreateSphere(const TATVector3& ct, float radius, float invMass)
{
	TATCollideShapeSphere* sphere = new TATCollideShapeSphere(ct, radius, invMass);

	//TATRigidBodyData* rbdata = m_RigidBodyDatas.FetchUnused();
	//TATInertiaData* in = m_InertiaDatas.FetchUnused();
	TATRigidBody* rb = m_RigidBodys.FetchUnused();
	rb->SetCollideShape(sphere);

	//rb->m_DataIndex = rbdata->m_IndexInPool;
	//rb->m_InertiaIndex = in->m_IndexInPool;

	return rb;
}

void TATDynamicWorld::DestroyRigidBody(TATRigidBody* rb)
{
	if (rb == 0)
		return;
	//m_InertiaDatas.ReturnUsed(rb->m_InertiaIndex);
	//m_RigidBodyDatas.ReturnUsed(rb->m_DataIndex);
	m_RigidBodys.ReturnUsed(rb->m_IndexInPool);
}

void TATDynamicWorld::InitRigidBody(TATRigidBody* rb, const TATransform& tr, float invMass, float restituitionCoeff, float frictionCoeff, const TATVector3& g)
{
	rb->SetWorldTransform(tr);
	rb->m_InvMass = invMass;

	//TATRigidBodyData& rbdata = m_RigidBodyDatas[rb->m_DataIndex];
	//TATInertiaData& in = m_InertiaDatas[rb->m_InertiaIndex];

	rb->m_Pos = rb->GetMassCenter(); // tr.GetOrigin();
	rb->m_Quat = tr.GetRotation();
	rb->m_LinVel = rb->m_AngVel = TATVector3::Zero();
	rb->m_FrictionCoeff = frictionCoeff;
	rb->m_RestituitionCoeff = restituitionCoeff;
	rb->m_InvMass = invMass;
	rb->m_Gravity = g;

	rb->m_InitInvInertia = rb->m_CollideShape->m_LocalInvInertiaTensor;
	//in.m_InvInertiaWorld = in.m_InitInvInertia * tr.GetBasis();

	rb->UpdataInverseInertiaWorld();
}


void TATDynamicWorld::SyncRigidBodyData(TATRigidBody* rb)
{
	rb->m_WorldTransform.SetOrigin(rb->m_Pos);
	rb->m_WorldTransform.SetRotation(rb->m_Quat);

	rb->UpdataInverseInertiaWorld();
}