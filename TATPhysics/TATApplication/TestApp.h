#pragma once
#include "TATApplication.h"

class TATLinePainter;

class TestApp :public TATApplication
{
public:
	TestApp(const TString& name) :TATApplication(name)
	{}

	virtual void Initialize() override;

	virtual void CreateScene() override;

	virtual void Run() override;

	virtual void BeginRenderOneFrame(float dt);

	virtual void RenderOneFrameEnd(float dt) {}

	virtual void SimulationStart(float dt) {}

	virtual void SimulationEnd(float dt) {}

	TATLinePainter* m_LinePainter;
};