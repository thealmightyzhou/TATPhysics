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

	TATMesh* mesh = TATResourceManager::Instance()->LoadMesh("cube.obj");

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("testMat.tmaterial");

	TATActor* actor = new TATActor(mesh);
	actor->SetMaterial(mat);

	TATActor* actor1 = new TATActor(mesh);
	actor1->SetMaterial(mat);

	TATStageNode* node = m_RootNode->CreateChild("cube1");
	node->MountActor(actor);
	node->SetVisible(true);
	node->SetTransform(TATransform::GetIdentity());

	TATStageNode* node2 = m_RootNode->CreateChild("cube2");
	node2->MountActor(actor1);
	node2->SetVisible(true);
	node2->SetTransform(TATransform::GetIdentity());
	
	TATRigidBody* rb0 = TATDynamicWorld::Instance()->CreateConvex(mesh, 1.0f);
	TATransform tr;
	tr.SetOrigin(TATVector3(0, 50, 10));
	TATDynamicWorld::Instance()->InitializeRb(rb0, tr, 1.0f, 0.8, 0.2, TATVector3(0, -9, 0));

	TATRigidBody* rb1 = TATDynamicWorld::Instance()->CreateConvex(mesh, 1.0f);
	tr.SetOrigin(TATVector3(0, 20, 20));
	TATDynamicWorld::Instance()->InitializeRb(rb1, tr, 2.0f, 0.8, 0.2, TATVector3(0, -4, 0));

	actor->SetRigidBody(rb0->m_IndexInPool);
	actor1->SetRigidBody(rb1->m_IndexInPool);

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