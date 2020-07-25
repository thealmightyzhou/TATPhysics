#pragma once

class TATRenderListener
{
public:
	TATRenderListener();
	virtual ~TATRenderListener() {}

	virtual void BeginRenderOneFrame(float dt) {}

	virtual void RenderOneFrameEnd(float dt) {}
};

class TATPhysicListener
{
public:
	TATPhysicListener();

	virtual ~TATPhysicListener() {}

	virtual void SimulationStart(float dt) {}

	virtual void SimulationEnd(float dt) {}
};