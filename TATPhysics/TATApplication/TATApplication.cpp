#include "TATApplication.h"
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATResources/TATResourceManager.h"
#include "TAThread.h"

TATApplication::TATApplication()
{

}

const TString& TATApplication::GetAppName()
{
	return m_AppName;
}