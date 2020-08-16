#include "TATApplication.h"
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATResources/TATResourceManager.h"
#include "TAThread.h"
#include "../TATStage/TATStageNode.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"

TATApplicationEntry* TATApplicationEntry::m_Entry = 0;

TATApplication::TATApplication(const TString& name):m_AppName(name)
{
	m_PhysicThread = 0;
	m_RenderThread = 0;
	m_RootNode = new TATStageNode("root");
	TATApplicationEntry::Instance()->SetApplication(this);
}

const TString& TATApplication::GetAppName()
{
	return m_AppName;
}

TATStageNode* TATApplication::GetRootNode()
{
	return m_RootNode;
}

void TATApplication::Initialize()
{
	m_PhysicThread = new TATPhysicThread;
	m_RenderThread = new TATRenderThread;

	m_RenderThread->AddListener(this);
	m_PhysicThread->AddListener(this);

	m_MainCamera = new TATCamera("main");
	m_MainCamera->Initialize();

	m_MainLight = new TATLight("main");
	m_MainLight->Initialize();
}

TATRenderThread* TATApplicationEntry::GetRenderThread()
{
	return Instance()->m_App->m_RenderThread;
}

TATPhysicThread* TATApplicationEntry::GetPhysicThread()
{
	return Instance()->m_App->m_PhysicThread;
}