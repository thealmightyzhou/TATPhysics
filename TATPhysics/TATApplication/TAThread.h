#pragma once
#include <iostream>
#include <thread>
#include "../TATDynamics/TATDynamicWorld.h"

class TATRenderListener;
class TATPhysicListener;

using namespace std;

class TAThread
{
public:
	virtual ~TAThread()
	{}

	void Run()
	{
		if(m_Thread.joinable())
			m_Thread.join();
	}

	thread m_Thread;
};

class TATPhysicThread :public TAThread
{
public:
	TATPhysicThread()
	{
		m_Thread = thread(this->PhysicLoop);
	}

	void PhysicLoop();

	void AddListener(TATPhysicListener* listener);

	void RemoveListener(TATPhysicListener* listener);

	std::vector<TATPhysicListener*> m_PhysicListeners;
};

class TATRenderThread :public TAThread
{
public:
	TATRenderThread()
	{
		m_Thread = thread(this->RenderLoop);
	}

	void RenderLoop();

	void RenderOneFrame()
	{
		//TODO

		m_RenderStateDirty = false;
	}

	//TODO fetch data from physics thread

	void AddListener(TATRenderListener* listener);

	void RemoverListener(TATRenderListener* listener);

	void MarkRenderStateDirty()
	{
		m_RenderStateDirty = true;
	}

	bool m_RenderStateDirty;

	std::vector<TATRenderListener*> m_RenderListeners;
};