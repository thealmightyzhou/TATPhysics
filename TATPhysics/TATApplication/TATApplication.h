#pragma once
#include "../TATCommon/TATSingleton.h"
#include "../TATResources/TATPaths.h"

class TString;

class TAThread;

class TATApplication:public Singleton<TATApplication>
{
public:
	TATApplication();

	virtual ~TATApplication() {}

	virtual void Initialize() {}

	virtual void CreateScene() {}

	//global entry
	virtual void Run();

	const TString& GetAppName();

	TString m_AppName;

	TAThread* m_PhysicThread;
	TAThread* m_RenderThread;
};