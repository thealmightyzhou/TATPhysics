#pragma once
#include "TATApplication.h"
#include "TATWorldListener.h"

class TATCamera;

class TestApp :public TATApplication,TATRenderListener,TATPhysicListener
{
public:
	TestApp(const TString& name) :TATApplication()
	{
		m_AppName = name;
	}

	virtual void Initialize();

	virtual void CreateScene();

	virtual void BeginRenderOneFrame(float dt) override;

	virtual void RenderOneFrameEnd(float dt) override
	{}

	virtual void SimulationStart(float dt) override
	{}

	virtual void SimulationEnd(float dt) override
	{}

	TATCamera* m_MainCamera;
};