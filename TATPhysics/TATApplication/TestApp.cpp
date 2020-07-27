#include "TestApp.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATGLEntry.h"
#include "../TATResources/TATResourceManager.h"
#include "TAThread.h"

void TestApp::Initialize()
{
	__super::Initialize();

	TATGLEntry::Instance()->Initialize(1080, 720);

	shared_ptr<TATMesh> mesh = TATResourceManager::Instance()->LoadMesh("test.obj");

	//TODO Load resources;

	m_MainCamera = new TATCamera("main");
}

void TestApp::CreateScene()
{
	__super::CreateScene();

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
	m_PhysicThread->Run();
	m_RenderThread->Run();
}