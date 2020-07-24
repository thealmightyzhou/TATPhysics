#pragma once

class TATRenderListener
{
public:
	TATRenderListener();
	virtual ~TATRenderListener() {}

	virtual void BeginRenderOneFrame() {}

	virtual void RenderOneFrameEnd() {}
};

class TATPhysicListener
{
public:
	TATPhysicListener();

	virtual ~TATPhysicListener() {}

	virtual void SimulationStart() {}

	virtual void SimulationEnd() {}
};