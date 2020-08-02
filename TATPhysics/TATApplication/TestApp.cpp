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

	m_LinePainter = new TATLinePainter();

	//TODO Load resources;
}

void TestApp::CreateScene()
{
	__super::CreateScene();

	m_MainCamera->SetPosition(TATVector3(30, 0, 0));
	m_MainCamera->SetWindowSize(TATGLEntry::Instance()->m_WindowWidth, TATGLEntry::Instance()->m_WindowHeight);

	TATMesh* mesh = TATResourceManager::Instance()->LoadMesh("cube.obj");

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("testMat.tmaterial");

	TATActor* actor = new TATActor(mesh);
	actor->SetMaterial(mat);

	TATStageNode* node = m_RootNode->CreateChild("test");

	node->MountActor(actor);
	node->SetVisible(true);
	node->SetTransform(TATransform::GetIdentity());
	
	TATRigidBody* rb0 = TATDynamicWorld::Instance()->CreateConvex(mesh, 1.0f);
	TATransform tr;
	tr.SetOrigin(TATVector3(0, 50, 10));
	TATDynamicWorld::Instance()->InitializeRb(rb0, tr, 1.0f, 0.8, 0.2);

	TATRigidBody* rb1 = TATDynamicWorld::Instance()->CreateConvex(mesh, 1.0f);
	tr.SetOrigin(TATVector3(0, 20, 20));
	TATDynamicWorld::Instance()->InitializeRb(rb1, tr, 2.0f, 0.8, 0.2);

	actor->SetRigidBody(rb0->m_IndexInPool);

	//TODO put resources to world and set physic behavior coefficient
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
	m_LinePainter->PaintLine(TATVector3::Zero(), TATVector3::UnitY() * 50, TATVector3(1, 1, 0));

	m_LinePainter->PaintLine(TATVector3(20, 20, 20), TATVector3::UnitX() * 50, TATVector3(0, 1, 1));
}