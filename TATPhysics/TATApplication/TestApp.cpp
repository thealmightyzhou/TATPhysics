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

	m_MainCamera->SetPosition(TATVector3(20, 20, -300));
	m_MainCamera->SetWindowSize(TATGLEntry::Instance()->m_WindowWidth, TATGLEntry::Instance()->m_WindowHeight);
	m_MainCamera->SetDirection((TATVector3(0, 50, 10) - m_MainCamera->GetPosition()).Normalized());
	m_MainCamera->SetPlane(0.01, 100000);
	m_MainCamera->SetSpeed(100);

	TATMesh* cube = TATResourceManager::Instance()->LoadMesh("cube.obj");
	TATMesh* pyramid = TATResourceManager::Instance()->LoadMesh("pyramid.obj");
	TATMesh* plane = TATResourceManager::Instance()->LoadMesh("plane.obj");
	TATMesh* sphere = TATResourceManager::Instance()->LoadMesh("sphere.obj");

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("testMat.tmaterial");

	TATransform tr;

	int row = 0;
	int col = 0;
	while (row < 11)
	{
		int x = col * 60;
		int y = row * 60;

		tr.SetOrigin(TATVector3(x, y, 10));

		TATActor* actor = new TATActor(cube);
		actor->SetMaterial(mat);
		TATStageNode* node = m_RootNode->CreateChild(TString("cube") + TString::ConvertInt(row) + TString::ConvertInt(col));
		node->MountActor(actor);

		TATRigidBody* rb = TATDynamicWorld::Instance()->CreateConvex(cube, 0.1f);

		TATDynamicWorld::Instance()->InitRigidBody(rb, tr, 0.1f, 0.8, 0.2, TATVector3(0, -100, 0));
		actor->SetRigidBody(rb->m_IndexInPool);

		col++;
		if (col > 5)
		{
			col = 0;
			row++;
		}
	}

	//TATActor* actor1 = new TATActor(pyramid);
	//actor1->SetMaterial(mat);
	//TATStageNode* node2 = m_RootNode->CreateChild("pyramid");
	//node2->MountActor(actor1);
	//TATRigidBody* rb1 = TATDynamicWorld::Instance()->CreateConvex(pyramid, 1.0f);
	//tr.SetOrigin(TATVector3(0, 20, 20));
	//TATDynamicWorld::Instance()->InitRigidBody(rb1, tr, 0.1f, 0.8, 0.2, TATVector3(0, -10, 0));
	//actor1->SetRigidBody(rb1->m_IndexInPool);

	/*TATActor* actor = new TATActor(cube);
	actor->SetMaterial(mat);
	TATStageNode* node = m_RootNode->CreateChild("cube1");
	node->MountActor(actor);
	TATRigidBody* rb0 = TATDynamicWorld::Instance()->CreateConvex(cube, 0.01f);
	tr.SetOrigin(TATVector3(0, 50, 10));
	TATDynamicWorld::Instance()->InitRigidBody(rb0, tr, 0.1f, 0.8, 0.2, TATVector3(0, -50, 0));
	actor->SetRigidBody(rb0->m_IndexInPool);

	TATActor* actor2 = new TATActor(cube);
	actor2->SetMaterial(mat);
	TATStageNode* node3 = m_RootNode->CreateChild("cube2");
	node3->MountActor(actor2);
	TATRigidBody* rb2 = TATDynamicWorld::Instance()->CreateConvex(cube, 1.0f);
	tr.SetOrigin(TATVector3(0, 80, 10));
	TATDynamicWorld::Instance()->InitRigidBody(rb2, tr, 0.1f, 0.8, 0.2, TATVector3(0, -50, 0));
	actor2->SetRigidBody(rb2->m_IndexInPool);

	TATActor* sphereActor = new TATActor(sphere);
	sphereActor->SetMaterial(mat);
	TATStageNode* sphereNode = m_RootNode->CreateChild("sphere");
	sphereNode->MountActor(sphereActor);
	TATRigidBody* sphereRb = TATDynamicWorld::Instance()->CreateSphere(TATVector3(0, 0, 0), 30, 1.0f);
	TATDynamicWorld::Instance()->InitRigidBody(sphereRb, TATransform(TATQuaternion::GetIdentity(), TATVector3(0, 50, 50)), 0.05f, 0.8, 0.2, TATVector3(0, -100, 0));
	sphereActor->SetRigidBody(sphereRb->m_IndexInPool);

	TATActor* sphereActor1 = new TATActor(sphere);
	sphereActor1->SetMaterial(mat);
	TATStageNode* sphereNode1 = m_RootNode->CreateChild("sphere1");
	sphereNode1->MountActor(sphereActor1);
	TATRigidBody* sphereRb1 = TATDynamicWorld::Instance()->CreateSphere(TATVector3(0, 0, 0), 30, 1.0f);
	TATDynamicWorld::Instance()->InitRigidBody(sphereRb1, TATransform(TATQuaternion::GetIdentity(), TATVector3(0, 200, 50)), 0.05f, 0.8, 0.2, TATVector3(0, -100, 0));
	sphereActor1->SetRigidBody(sphereRb1->m_IndexInPool);*/


	TATActor* planeActor = new TATActor(plane);
	planeActor->SetMaterial(mat);
	TATStageNode* planeNode = m_RootNode->CreateChild("plane");
	planeNode->MountActor(planeActor);
	TATDynamicWorld::Instance()->CreatePlane(TATVector3(0, -300, 0), TATVector3(0, 1, 0));

	TATActor* planeActor1 = new TATActor(plane);
	planeActor1->SetMaterial(mat);
	TATStageNode* planeNode1 = m_RootNode->CreateChild("plane1");
	planeNode1->MountActor(planeActor1);
	TATDynamicWorld::Instance()->CreatePlane(TATVector3(500, -300, 0), TATVector3(-1, 0, 0));

	TATActor* planeActor2 = new TATActor(plane);
	planeActor2->SetMaterial(mat);
	TATStageNode* planeNode2 = m_RootNode->CreateChild("plane2");
	planeNode2->MountActor(planeActor2);
	TATDynamicWorld::Instance()->CreatePlane(TATVector3(-200, -300, 0), TATVector3(1, 0, 0));

	TATActor* planeActor3 = new TATActor(plane);
	planeActor3->SetMaterial(mat);
	TATStageNode* planeNode3 = m_RootNode->CreateChild("plane3");
	planeNode3->MountActor(planeActor3);
	TATDynamicWorld::Instance()->CreatePlane(TATVector3(0, -300, 200), TATVector3(0, 0, -1));

	TATActor* planeActor4 = new TATActor(plane);
	planeActor4->SetMaterial(mat);
	TATStageNode* planeNode4 = m_RootNode->CreateChild("plane4");
	planeNode4->MountActor(planeActor4);
	TATDynamicWorld::Instance()->CreatePlane(TATVector3(0, -300, -200), TATVector3(0, 0, 1));

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