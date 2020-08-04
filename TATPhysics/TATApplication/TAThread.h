#pragma once
#include <iostream>
#include <thread>
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATCommon/TATSingleton.h"
#include "../TATCommon/TATObjectPool.h"
#include "../TATGLRender/TATGLRenderer.h"
#include "../TATGLRender/TATGLEntry.h"
#include "TATRenderTask.h"
#include "../TATStage/TATLinePainter.h"
#include <mutex>

class TATRenderListener;
class TATPhysicListener;
class TATRenderUnit;
class TATRigidBodyData;
class TATInertiaData;

using namespace std;

#define TAT_MAX_RENDERUNIT_COUNT 100
#define TAT_MAX_RIGIDBODY£ßCOUNT 100

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

class TATPhysicThread :public TAThread
{
public:
	TATPhysicThread()
	{
		
	}

	virtual void Run() override
	{
		m_Thread = thread(&TATPhysicThread::PhysicLoop, this);
		if (m_Thread.joinable())
			m_Thread.detach();
	}

	void PhysicLoop();

	void AddListener(TATPhysicListener* listener);

	void RemoveListener(TATPhysicListener* listener);

	std::vector<TATPhysicListener*> m_PhysicListeners;
};

//opengl function only used in main thread?
class TATRenderThread// :public TAThread
{
public:
	TATRenderThread(): m_RenderUnitPool(TAT_MAX_RENDERUNIT_COUNT)
	{
		m_Renderer = new TATGLRenderer();

		m_RenderStateDirty = true;

		m_Window = 0;
	}

	virtual void Run()
	{
		m_LinePainter = new TATLinePainter();

		RenderLoop();
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

	GLFWwindow* m_Window;

	TATRenderTaskList m_RenderTaskList;

	TATLinePainter* m_LinePainter;
};