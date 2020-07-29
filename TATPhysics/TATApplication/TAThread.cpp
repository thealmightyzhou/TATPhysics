#include "TAThread.h"
#include "TATWorldListener.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATGLRender/TATGLRenderer.h"
#include "../TATBasis/TATWorld.h"
#include "../TATStage/TATStageNode.h"
#include "../TATStage/TATActor.h"

void TATPhysicThread::AddListener(TATPhysicListener* listener)
{
	m_PhysicListeners.push_back(listener);
}

void TATPhysicThread::RemoveListener(TATPhysicListener* listener)
{
	std::vector<TATPhysicListener*>::iterator it = m_PhysicListeners.begin();
	for (; it != m_PhysicListeners.end(); it++)
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

		for (int i = 0; i < (int)m_PhysicListeners.size(); ++i)
		{
			m_PhysicListeners[i]->SimulationStart(dt);
		}

		TATDynamicWorld::Instance()->StepSimulation(dt);

		//TODO lock , fill the render buffer and mark as dirty

		for (int i = 0; i < (int)m_PhysicListeners.size(); ++i)
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
	for (; it != m_RenderListeners.end(); it++)
	{
		if (*it == listener)
			m_RenderListeners.erase(it);
	}
}

void TATRenderThread::RenderLoop()
{
	if(!m_Window)
		m_Window = TATGLEntry::Instance()->GetWindow();

	float dt = float(1) / 60;
	while (!glfwWindowShouldClose(m_Window))
	{
		m_RenderStateDirty = true;

		if (m_RenderStateDirty)
		{
			glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			for (int i = 0; i < (int)m_RenderListeners.size(); ++i)
			{
				m_RenderListeners[i]->BeginRenderOneFrame(dt);
			}

			RenderOneFrame(dt);

			glfwSwapBuffers(m_Window);
			glfwPollEvents();

			for (int i = 0; i < (int)m_RenderListeners.size(); ++i)
			{
				m_RenderListeners[i]->RenderOneFrameEnd(dt);
			}

		}
	}
}

void TATRenderThread::RenderOneFrame(float dt)
{
	std::map<TString, TATStageNode*>& renderNodes = TATWorld::Instance()->m_StageNodes;
	std::map<TString, TATStageNode*>::iterator ite = renderNodes.begin();
	TATStageNode* node = 0;
	TATActor* actor = 0;
	TATRenderUnit* unit = 0;
	for (; ite != renderNodes.end(); ite++)
	{
		node = ite->second;
		if(node)
			actor = node->m_Actor;
		if (actor)
			unit = actor->m_RenderUnit;

		if (node && node->GetVisible() && actor && unit)
		{
			actor->FillRenderUnit();
			if(unit->m_ReadyToRender)
				m_Renderer->Render(unit);
		}

		node = 0;
		actor = 0;
		unit = 0;
	}

	m_RenderStateDirty = false;
}