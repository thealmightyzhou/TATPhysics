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
#include "../TATScripted/RigidBodyPawn.h"

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

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("LightTemplate.tmaterial");
	mat->Initialize();

	TATActor* cubeActor = new TATActor(cube);
	cubeActor->Initialize();
	cubeActor->SetMaterial(mat);
	cubeActor->SetUseTransform(true);
	TATStageNode* node3 = m_RootNode->CreateChild("cube");
	node3->MountActor(cubeActor);
	TATRigidBody* cubeRb = TATDynamicWorld::Instance()->CreateConvex(cube, 1.0f);
	TATransform tr;
	tr.SetOrigin(TATVector3(0, -200, 10));
	//tr.SetRotation(TATQuaternion(0, 0.8, 0.6, 0));
	TATDynamicWorld::Instance()->InitRigidBody(cubeRb, tr, 0.1f, 0.8, 0.2, TATVector3(0, -10, 0));
	cubeActor->AttachTickable(cubeRb);

	TATActor* cubeActor1 = new TATActor(cube);
	cubeActor1->Initialize();
	cubeActor1->SetMaterial(mat);
	cubeActor1->SetUseTransform(true);
	TATStageNode* node4 = m_RootNode->CreateChild("cube1");
	node4->MountActor(cubeActor1);
	TATRigidBody* cubeRb1 = TATDynamicWorld::Instance()->CreateConvex(cube, 1.0f);
	tr.SetOrigin(TATVector3(0, 100, 10));
	//tr.SetRotation(TATQuaternion(0, 0.8, 0.6, 0));
	TATDynamicWorld::Instance()->InitRigidBody(cubeRb1, tr, 1.0, 0.8, 0.2, TATVector3(0, -10, 0));
	cubeActor1->AttachTickable(cubeRb1);

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
	softBody0->AddPositionConstraint(0.8, 0.8);
	softBody0->AddVolumeConstraint(0.9, 0.9);
	softBody0->SetDamping(0.8, 0.8);
	softBody0->SetGravity(TATVector3(0, -50, 0));
	softActor0->AttachTickable(softBody0);


	TATRigidBody* pawnRb = TATDynamicWorld::Instance()->CreateConvex(cube, 1);
	tr.SetOrigin(TATVector3(0, 500, 10));
	TATDynamicWorld::Instance()->InitRigidBody(pawnRb, tr, 0, 0.8, 0.2, TATVector3(0, 0, 0));
	RigidBodyPawn* cubePawn = new RigidBodyPawn(cube, pawnRb);
	cubePawn->Initialize();
	cubePawn->SetMaterial(mat);
	cubePawn->SetUseTransform(true);
	TATStageNode* pawnNode = m_RootNode->CreateChild("pawn");
	pawnNode->MountActor(cubePawn);
	cubePawn->AttachTickable(pawnRb);


	//====================
	//TATQuaternion rot;

	//TATActor* planeActor1 = new TATActor(plane);
	//planeActor1->Initialize();
	//planeActor1->SetMaterial(mat);
	//tr.SetOrigin(TATVector3(500, -300, 0));
	//rot.FromAngleAxis(TATVector3::UnitZ(), TAT_HALF_PI);
	//tr.SetRotation(rot);
	//planeActor1->m_WorldTransform = tr;
	//planeActor1->SetUseTransform(true);
	//TATStageNode* planeNode1 = m_RootNode->CreateChild("plane1");
	//planeNode1->MountActor(planeActor1);
	//TATDynamicWorld::Instance()->CreatePlane(TATVector3(500, -300, 0), TATVector3(-1, 0, 0));

	//TATActor* planeActor2 = new TATActor(plane);
	//planeActor2->Initialize();
	//planeActor2->SetMaterial(mat);
	//tr.SetOrigin(TATVector3(-200, -300, 0));
	//rot.FromAngleAxis(TATVector3::UnitZ(), -TAT_HALF_PI);
	//tr.SetRotation(rot);
	//planeActor2->m_WorldTransform = tr;
	//planeActor2->SetUseTransform(true);
	//TATStageNode* planeNode2 = m_RootNode->CreateChild("plane2");
	//planeNode2->MountActor(planeActor2);
	//TATDynamicWorld::Instance()->CreatePlane(TATVector3(-200, -300, 0), TATVector3(1, 0, 0));

	//TATActor* planeActor3 = new TATActor(plane);
	//planeActor3->Initialize();
	//planeActor3->SetMaterial(mat);
	//tr.SetOrigin(TATVector3(0, -300, 200));
	//rot.FromAngleAxis(TATVector3::UnitX(), TAT_HALF_PI);
	//tr.SetRotation(rot);
	//planeActor3->m_WorldTransform = tr;
	//planeActor3->SetUseTransform(true);
	//TATStageNode* planeNode3 = m_RootNode->CreateChild("plane3");
	//planeNode3->MountActor(planeActor3);
	//TATDynamicWorld::Instance()->CreatePlane(TATVector3(0, -300, 200), TATVector3(0, 0, -1));

	//TATActor* planeActor4 = new TATActor(plane);
	//planeActor4->Initialize();
	//planeActor4->SetMaterial(mat);
	//tr.SetOrigin(TATVector3(0, -300, -200));
	//rot.FromAngleAxis(TATVector3::UnitX(), -TAT_HALF_PI);
	//tr.SetRotation(rot);
	//planeActor4->m_WorldTransform = tr;
	//planeActor4->SetUseTransform(true);
	//TATStageNode* planeNode4 = m_RootNode->CreateChild("plane4");
	//planeNode4->MountActor(planeActor4);
	//TATDynamicWorld::Instance()->CreatePlane(TATVector3(0, -300, -200), TATVector3(0, 0, 1));
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
