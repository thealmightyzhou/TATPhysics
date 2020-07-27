#pragma once
#include <iostream>
#include <thread>
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATCommon/TATSingleton.h"
#include "../TATCommon/TATObjectPool.h"
#include "../TATGLRender/TATGLRenderer.h"

class TATRenderListener;
class TATPhysicListener;
class TATRenderUnit;

using namespace std;

#define TAT_MAX_RENDERUNIT_COUNT 100

class TAThread
{
public:
	virtual ~TAThread()
	{}

	virtual void Run()
	{
		if(m_Thread.joinable())
			m_Thread.join();
	}

	thread m_Thread;
};

class TATPhysicThread :public TAThread,public Singleton<TATPhysicThread>
{
public:
	TATPhysicThread()
	{
		
	}

	virtual void Run() override
	{
		m_Thread = thread(&TATPhysicThread::PhysicLoop, this);
		if (m_Thread.joinable())
			m_Thread.join();
	}

	void PhysicLoop();

	void AddListener(TATPhysicListener* listener);

	void RemoveListener(TATPhysicListener* listener);

	std::vector<TATPhysicListener*> m_PhysicListeners;
};

class TATRenderThread :public TAThread,public Singleton<TATRenderThread>
{
public:
	TATRenderThread(): m_RenderUnitPool(TAT_MAX_RENDERUNIT_COUNT)
	{
		m_Renderer = new TATGLRenderer();
	}

	virtual void Run() override
	{
		m_Thread = thread(&TATRenderThread::RenderLoop, this);
		if (m_Thread.joinable())
			m_Thread.join();
	}

	void RenderLoop();

	void RenderOneFrame(float dt);

	//TODO fetch data from physics thread

	void AddListener(TATRenderListener* listener);

	void RemoverListener(TATRenderListener* listener);

	void MarkRenderStateDirty()
	{
		m_RenderStateDirty = true;
	}

	bool m_RenderStateDirty;

	std::vector<TATRenderListener*> m_RenderListeners;

	TATObjectPool<TATRenderUnit> m_RenderUnitPool;

	TATGLRenderer* m_Renderer;
};