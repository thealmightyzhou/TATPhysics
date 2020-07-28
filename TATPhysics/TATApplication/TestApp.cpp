#include "TestApp.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATGLEntry.h"
#include "../TATResources/TATResourceManager.h"
#include "../TATStage/TATStageNode.h"
#include "TAThread.h"
#include "../TATStage/TATActor.h"
#include "../TATResources/TATMaterial.h"
#include "../TATResources/TATexture.h"

void TestApp::Initialize()
{
	__super::Initialize();

	TATGLEntry::Instance()->Initialize(1080, 720);

	//TODO Load resources;
}

void TestApp::CreateScene()
{
	__super::CreateScene();

	TATMesh* mesh = TATResourceManager::Instance()->LoadMesh("cube.obj");

	TATMaterial* mat = TATResourceManager::Instance()->LoadMaterial("testMat.tmaterial");

	TATActor* actor = new TATActor(mesh);
	actor->SetMaterial(mat);

	TATStageNode* node = m_RootNode->CreateChild("test");

	node->MountActor(actor);
	node->SetVisible(true);
	node->SetTransform(TATransform::GetIdentity());

	//TODO put resources to world and set physic behavior coefficient
}

void TestApp::BeginRenderOneFrame(float dt)
{
	m_MainCamera->Move(dt);
}

void TestApp::Run()
{
	Initialize();

	CreateScene();

	m_PhysicThread = new TATPhysicThread;
	m_RenderThread = new TATRenderThread;
	//m_PhysicThread->Run();
	m_RenderThread->Run();
}