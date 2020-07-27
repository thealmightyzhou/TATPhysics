#pragma once
#include "../TATCommon/TATSingleton.h"
#include "../TATResources/TATPaths.h"

class TAThread;
class TATStageNode;

#define TAT_ROOTNODE TATApplication::Instance()->GetRootNode()

class TATApplication:public Singleton<TATApplication>
{
public:
	TATApplication();

	virtual ~TATApplication() {}

	virtual void Initialize() {}

	virtual void CreateScene() {}

	//global entry
	virtual void Run() {}

	const TString& GetAppName();

	TATStageNode* GetRootNode();

	TString m_AppName;

	TAThread* m_PhysicThread;
	TAThread* m_RenderThread;

	TATStageNode* m_RootNode;
};