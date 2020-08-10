#include "TestApp.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATGLEntry.h"
#include "../TATResources/TATResourceManager.h"
#include "../TATStage/TATStageNode.h"
#include "TAThread.h"
#include "../TATStage/TATActor.h"
#include "../TATResources/TATMaterial.h"
#include "../TATResources/TATexture.h"
#include "../TATStage/TATLinePainter.h"

void TestApp::Initialize()
{
	__super::Initialize();

	TATGLEntry::Instance()->Initialize(1080, 720);
}

void TestApp::CreateScene()
{
	__super::CreateScene();

	m_MainCamera->SetPosition(TATVector3(20, 20, -100));
	m_MainCamera->SetWindowSize(TATGLEntry::Instance()->m_WindowWidth, TATGLEntry::Instance()->m_WindowHeight);
	m_MainCamera->SetDirection((TATVector3(0, 50, 10) - m_MainCamera->GetPosition()).Normalized());
	m_MainCamera->SetPlane(0.01, 100000);

	TATMesh* cube = TATResourceManager::Instance()->LoadMesh("cube.obj");
	TATMesh* pyramid = TATResourceManager::Instance()->LoadMesh("pyramid.obj");
	TATMesh* plane = TATResourceManager::Instance()->LoadMesh("plane.obj");
	TATMesh* sphere = TATResourceManager::Instance()->LoadMesh("sphere.obj");

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("testMat.tmaterial");

	TATActor* actor = new TATActor(cube);
	actor->SetMaterial(mat);
	TATStageNode* node = m_RootNode->CreateChild("cube1");
	node->MountActor(actor);
	TATRigidBody* rb0 = TATDynamicWorld::Instance()->CreateConvex(cube, 0.01f);
	TATransform tr;
	tr.SetOrigin(TATVector3(0, 50, 10));
	TATDynamicWorld::Instance()->InitRigidBody(rb0, tr, 0.1f, 0.8, 0.2, TATVector3(0, -9, 0));
	actor->SetRigidBody(rb0->m_IndexInPool);


	//TATActor* actor1 = new TATActor(pyramid);
	//actor1->SetMaterial(mat);
	//TATStageNode* node2 = m_RootNode->CreateChild("pyramid");
	//node2->MountActor(actor1);
	//TATRigidBody* rb1 = TATDynamicWorld::Instance()->CreateConvex(pyramid, 1.0f);
	//tr.SetOrigin(TATVector3(0, 20, 20));
	//TATDynamicWorld::Instance()->InitRigidBody(rb1, tr, 0.1f, 0.8, 0.2, TATVector3(0, -10, 0));
	//actor1->SetRigidBody(rb1->m_IndexInPool);


	//TATActor* actor2 = new TATActor(cube);
	//actor2->SetMaterial(mat);
	//TATStageNode* node3 = m_RootNode->CreateChild("cube3");
	//node3->MountActor(actor2);
	//TATRigidBody* rb2 = TATDynamicWorld::Instance()->CreateConvex(cube, 1.0f);
	//tr.SetOrigin(TATVector3(0, 80, 20));
	//TATDynamicWorld::Instance()->InitRigidBody(rb2, tr, 0.1f, 0.8, 0.2, TATVector3(0, -15, 0));
	//actor2->SetRigidBody(rb2->m_IndexInPool);


	//TATActor* sphereActor = new TATActor(sphere);
	//sphereActor->SetMaterial(mat);
	//TATStageNode* sphereNode = m_RootNode->CreateChild("sphere");
	//sphereNode->MountActor(sphereActor);
	//TATRigidBody* sphereRb = TATDynamicWorld::Instance()->CreateSphere(TATVector3(0, 0, 0), 30, 1.0f);
	//TATDynamicWorld::Instance()->InitRigidBody(sphereRb, TATransform(TATQuaternion::GetIdentity(), TATVector3(0, 50, 50)), 0.05f, 0.8, 0.2, TATVector3(0, -10, 0));
	//sphereActor->SetRigidBody(sphereRb->m_IndexInPool);


	TATActor* planeActor = new TATActor(plane);
	planeActor->SetMaterial(mat);
	TATStageNode* planeNode = m_RootNode->CreateChild("plane");
	planeNode->MountActor(planeActor);
	TATDynamicWorld::Instance()->CreatePlane(TATVector3(0, -300, 0), TATVector3(0, 1, 0));

}

void TestApp::Run()
{
	Initialize();

	CreateScene();

	m_PhysicThread->Run();

	m_RenderThread->Run();
}

void TestApp::BeginRenderOneFrame(float dt)
{

}