#pragma once

#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATResources/TATResourceManager.h"
#include "TAThread.h"

class TATApplication:public Singleton<TATApplication>
{
public:
	TATApplication(const TString& name) :m_AppName(name)
	{
		m_PhysicThread = new TATPhysicThread;
		m_RenderThread = new TATRenderThread;
	}

	virtual ~TATApplication() {}

	virtual void Initialize() {}

	virtual void CreateScene() {}

	//global entry
	virtual void Run()
	{
		Initialize();

		CreateScene();

		m_PhysicThread->Run();
		m_RenderThread->Run();
	}

	const TString& GetAppName()
	{
		return m_AppName;
	}

	TString m_AppName;

	TAThread* m_PhysicThread;
	TAThread* m_RenderThread;
};