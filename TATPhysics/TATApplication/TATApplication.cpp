#include "TATApplication.h"
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATResources/TATResourceManager.h"
#include "TAThread.h"
#include "../TATStage/TATStageNode.h"

TATApplication::TATApplication()
{

}

const TString& TATApplication::GetAppName()
{
	return m_AppName;
}

TATStageNode* TATApplication::GetRootNode()
{
	return m_RootNode;
}