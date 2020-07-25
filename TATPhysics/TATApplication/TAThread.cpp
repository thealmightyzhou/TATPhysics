#include "TAThread.h"
#include "TATWorldListener.h"

void TATPhysicThread::AddListener(TATPhysicListener* listener)
{
	m_PhysicListeners.push_back(listener);
}

void TATPhysicThread::RemoveListener(TATPhysicListener* listener)
{
	std::vector<TATPhysicListener*>::iterator it = m_PhysicListeners.begin();
	for (; it != m_PhysicListeners.end; it++)
	{
		if (*it == listener)
			m_PhysicListeners.erase(it);
	}
}

void TATPhysicThread::PhysicLoop()
{
	float dt = float(1) / 60;
	while (true)
	{

		for (int i = 0; i < m_PhysicListeners.size(); ++i)
		{
			m_PhysicListeners[i]->SimulationStart(dt);
		}

		TATDynamicWorld::Instance()->StepSimulation(dt);

		//TODO lock , fill the render buffer and mark as dirty

		for (int i = 0; i < m_PhysicListeners.size(); ++i)
		{
			m_PhysicListeners[i]->SimulationEnd(dt);
		}
	}
}

//============================

void TATRenderThread::AddListener(TATRenderListener* listener)
{
	m_RenderListeners.push_back(listener);
}

void TATRenderThread::RemoverListener(TATRenderListener* listener)
{
	std::vector<TATRenderListener*>::iterator it = m_RenderListeners.begin();
	for (; it != m_RenderListeners.end; it++)
	{
		if (*it == listener)
			m_RenderListeners.erase(it);
	}
}

void TATRenderThread::RenderLoop()
{
	float dt = float(1) / 60;
	while (true)
	{
		if (m_RenderStateDirty)
		{
			for (int i = 0; i < m_RenderListeners.size(); ++i)
			{
				m_RenderListeners[i]->BeginRenderOneFrame(dt);
			}

			RenderOneFrame(dt);

			//swap buffer

			for (int i = 0; i < m_RenderListeners.size(); ++i)
			{
				m_RenderListeners[i]->RenderOneFrameEnd(dt);
			}
		}
	}
}