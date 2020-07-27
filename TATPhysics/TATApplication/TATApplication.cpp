#include "TATApplication.h"
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATResources/TATResourceManager.h"
#include "TAThread.h"

TATApplication::TATApplication()
{
	m_PhysicThread = new TATPhysicThread;
	m_RenderThread = new TATRenderThread;
}

void TATApplication::Run()
{
	Initialize();

	CreateScene();

	m_PhysicThread->Run();
	m_RenderThread->Run();
}

const TString& TATApplication::GetAppName()
{
	return m_AppName;
}