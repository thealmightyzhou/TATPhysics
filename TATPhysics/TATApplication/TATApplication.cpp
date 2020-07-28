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
	m_MainCamera = new TATCamera("main");

	m_MainLight = new TATLight("main");
}