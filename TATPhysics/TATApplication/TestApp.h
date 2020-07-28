#pragma once
#include "TATApplication.h"
#include "TATWorldListener.h"

class TestApp :public TATApplication,TATRenderListener,TATPhysicListener
{
public:
	TestApp(const TString& name) :TATApplication(name)
	{}

	virtual void Initialize() override;

	virtual void CreateScene() override;

	virtual void Run() override;

	virtual void BeginRenderOneFrame(float dt) override;

	virtual void RenderOneFrameEnd(float dt) override
	{}

	virtual void SimulationStart(float dt) override
	{}

	virtual void SimulationEnd(float dt) override
	{}

};