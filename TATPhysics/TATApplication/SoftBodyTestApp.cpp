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
	TATMesh* cube = TATResourceManager::Instance()->LoadMesh("cube.obj");

	plane->Initialize();
	tet_halfSphere->Initialize();
	cube->Initialize();

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("testMat.tmaterial");
	mat->Initialize();

	TATActor* cubeActor = new TATActor(cube);
	cubeActor->Initialize();
	cubeActor->SetMaterial(mat);
	cubeActor->SetUseTransform(true);
	TATStageNode* node3 = m_RootNode->CreateChild("cube");
	node3->MountActor(cubeActor);
	TATRigidBody* cubeRb = TATDynamicWorld::Instance()->CreateConvex(cube, 1.0f);
	TATransform tr;
	tr.SetOrigin(TATVector3(0, -100, 10));
	tr.SetRotation(TATQuaternion(0, 0.8, 0.6, 0));
	TATDynamicWorld::Instance()->InitRigidBody(cubeRb, tr, 0.1f, 0.8, 0.2, TATVector3(0, 0, 0));
	cubeActor->AttachTickable(cubeRb);

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
	softBody0->AddVolumeConstraint(0.9, 0.9);
	softBody0->SetDamping(0.8, 0.8);
	softBody0->SetGravity(TATVector3(0, -100, 0));
	softActor0->AttachTickable(softBody0);

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
