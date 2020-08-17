#include "SoftBodyTestApp.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATGLEntry.h"
#include "../TATResources/TATResourceManager.h"
#include "../TATStage/TATStageNode.h"
#include "TAThread.h"
#include "../TATStage/TATActor.h"
#include "../TATResources/TATMaterial.h"
#include "../TATResources/TATexture.h"
#include "../TATStage/TATLinePainter.h"
#include "../TATPositionBasedDynamics/TATPBDBody.h"
#include "../TATPositionBasedDynamics/TATPBDWorld.h"
#include "../TATNarrowPhase/TATCollisionUtil.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"


class SoftPlaneOverlapCallBack :public TATBvhCollideCallBack
{
public:
	SoftPlaneOverlapCallBack(TATVector3 points[4], TATRigidBody* rb) :m_Rigid(rb)
	{
		for (int i = 0; i < 4; ++i)
		{
			m_PlanePoints[i] = points[i];
		}
	}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
	{
		TATPBDParticle* particle = (TATPBDParticle*)(node1->m_Data);
		int which = (int)(node2->m_Data);

		TATVector3 points[3];
		if (which == 0)
		{
			points[0] = m_PlanePoints[0];
			points[1] = m_PlanePoints[1];
			points[2] = m_PlanePoints[2];
		}
		else
		{
			points[0] = m_PlanePoints[0];
			points[1] = m_PlanePoints[2];
			points[2] = m_PlanePoints[3];
		}

		TATVector3 faceVel[3]{ TATVector3::Zero(),TATVector3::Zero() ,TATVector3::Zero() };
		TATVector3 vel = particle->m_PredictPos - particle->Position();

		float t;
		float margin = 0.2;

		if (TATCollisionUtil::PtCollideFaceContinous(particle->Position(), vel, points, faceVel, t, 10, margin))
		{
			TATVector3 p = particle->Position() + vel * t;
			//plane face is static
			TATVector3 norm = ((points[1] - points[0]).Cross(points[2] - points[0])).Normalized();
			float pen = margin - (p - m_PlanePoints[0]).Dot(norm);

			TATSoftRigidCollideData data;
			data.m_CollideNormal = norm;
			data.m_Particle = particle;
			data.m_Rigid = m_Rigid;
			data.m_SoftPt = p;
			data.m_Penetration = pen;
			data.m_RigidPt = p - norm * pen;

			TATPBDWorld::Instance()->AddSoftRigidCollideData(data);
		}
	}

	TATVector3 m_PlanePoints[4];
	TATRigidBody* m_Rigid;
};

class SoftCollidePlaneProcessor :public TATSoftRigidCollideListener
{
public:
	SoftCollidePlaneProcessor(TATRigidBody* plane, TATPBDBody* body) :m_Rigid(plane), m_SoftBody(body)
	{
		m_ColPlane = dynamic_cast<TATCollideShapePlane*>(plane->m_CollideShape);
	}

	virtual void GenerateCollision()
	{
		TATVector3 l1, l2;
		m_ColPlane->m_Normal.PlaneSpace(l1, l2);

		TATVector3 points[4];
		points[0] = m_ColPlane->m_Origin + l1 * 10000;
		points[1] = m_ColPlane->m_Origin + l2 * 10000;
		points[2] = m_ColPlane->m_Origin - l1 * 10000;
		points[3] = m_ColPlane->m_Origin - l2 * 10000;
		//0,1,2 ; 0,2,3

		TATBvh planeTree;
		TATVector3 min = points[0];
		min.SetMin(points[1]);
		min.SetMin(points[2]);
		TATVector3 max = points[0];
		max.SetMax(points[1]);
		max.SetMax(points[2]);

		TATBVNode* node = planeTree.InsertAabbNode(min - TATVector3::One() * 0.5f, max + TATVector3::One() * 0.5f);
		node->m_Data = (void*)(0); //if 0 take 0,1,2

		min = points[0];
		min.SetMin(points[2]);
		min.SetMin(points[3]);
		max = points[0];
		max.SetMax(points[2]);
		max.SetMax(points[3]);
		node = planeTree.InsertAabbNode(min - TATVector3::One() * 0.5f, max + TATVector3::One() * 0.5f);
		node->m_Data = (void*)(1);

		planeTree.FinishBuild();

		SoftPlaneOverlapCallBack cb(points, m_Rigid);
		planeTree.CollideWithBVTree(&m_SoftBody->m_ParticleBVH, &cb);
	}

	TATRigidBody* m_Rigid;
	TATCollideShapePlane* m_ColPlane;
	TATPBDBody* m_SoftBody;
};

void SoftBodyTestApp::Initialize()
{
	__super::Initialize();

	TATGLEntry::Instance()->Initialize(1080, 720);
}


void SoftBodyTestApp::CreateScene()
{
	__super::CreateScene();

	m_MainCamera->SetPosition(TATVector3(20, 20, -300));
	m_MainCamera->SetWindowSize(TATGLEntry::Instance()->m_WindowWidth, TATGLEntry::Instance()->m_WindowHeight);
	m_MainCamera->SetDirection((TATVector3(0, 50, 10) - m_MainCamera->GetPosition()).Normalized());
	m_MainCamera->SetPlane(0.01, 100000);
	m_MainCamera->SetSpeed(100);

	TATMesh* plane = TATResourceManager::Instance()->LoadMesh("plane.obj");
	TATMesh* tet_halfSphere = TATResourceManager::Instance()->LoadMesh("halfSphere.tmodel");

	plane->Initialize();
	tet_halfSphere->Initialize();

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("testMat.tmaterial");
	mat->Initialize();

	TATActor* planeActor = new TATActor(plane);
	planeActor->Initialize();
	planeActor->SetMaterial(mat);
	TATStageNode* planeNode = m_RootNode->CreateChild("plane");
	planeNode->MountActor(planeActor);
	TATRigidBody* planeRb = TATDynamicWorld::Instance()->CreatePlane(TATVector3(0, -300, 0), TATVector3(0, 1, 0));

	TATActor* softActor0 = new TATActor(tet_halfSphere);
	softActor0->Initialize();
	softActor0->SetMaterial(mat);
	TATStageNode* softNode0 = m_RootNode->CreateChild("softHalfSphere");
	softNode0->MountActor(softActor0);
	TATPBDBody* softBody0 = new TATPBDBody("softHalfSphere", tet_halfSphere->m_Loader->m_Buffer, 1.0f);
	softBody0->Initialize();
	softBody0->AddPositionConstraint(0.1, 0.1);
	softBody0->AddVolumeConstraint(0.1, 0.1);
	softBody0->SetDamping(0.8, 0.8);
	softBody0->SetGravity(TATVector3(0, -100, 0));
	softActor0->AttachTickable(softBody0);

	SoftCollidePlaneProcessor* processor = new SoftCollidePlaneProcessor(planeRb, softBody0);
	TATPBDWorld::Instance()->AddCollideProcessor(processor);
}

void SoftBodyTestApp::Run()
{
	Initialize();

	CreateScene();

	m_PhysicThread->Run();

	m_RenderThread->Run();
}

void SoftBodyTestApp::BeginRenderOneFrame(float dt)
{

}
