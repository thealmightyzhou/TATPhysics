#include "TAThread.h"
#include "TATWorldListener.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATGLRender/TATGLRenderer.h"
#include "../TATBasis/TATWorld.h"
#include "../TATStage/TATStageNode.h"
#include "../TATStage/TATActor.h"
#include "../TATBasis/TATErrorReporter.h"
#include "../TATBasis/TATimer.h"
#include "../TATPositionBasedDynamics/TATPBDWorld.h"

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
	float timeStep = float(1) / 100;
	float dt = 0;

	TATimer timer;

	int fps = 0;
	float timeCollector = 0;

	while (true)
	{
		timer.Begin();

		if (!timer.Block(dt, timeStep))
		{
			for (int i = 0; i < (int)m_PhysicListeners.size(); ++i)
			{
				m_PhysicListeners[i]->SimulationStart(timeStep);
			}

			TATPBDWorld::Instance()->StepSimulation(timeStep);

			TATDynamicWorld::Instance()->StepSimulation(timeStep);

			for (int i = 0; i < (int)m_PhysicListeners.size(); ++i)
			{
				m_PhysicListeners[i]->SimulationEnd(timeStep);
			}

			fps++;
			//TATErrorReporter::Instance()->ReportErr("physic one step: " + TString::ConvertFloat(timeStep));
		}

		timer.End();

		dt = timer.DeltaTime(TAT_MAX);

		timeCollector += dt;

		if (timeCollector > 1.0f)
		{
			TATErrorReporter::Instance()->ReportErr("step count: " + TString::ConvertInt(fps));
			timeCollector = 0;
			fps = 0;
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

	TATimer timer;

	float dt = float(1) / 60;
	while (!glfwWindowShouldClose(m_Window))
	{
		m_RenderStateDirty = true;

		timer.Begin();

		if (m_RenderStateDirty)
		{
			glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
			glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

			for (int i = 0; i < (int)m_RenderListeners.size(); ++i)
			{
				m_RenderListeners[i]->BeginRenderOneFrame(dt);
			}

			m_RenderTaskList.HandleTask();

			RenderOneFrame(dt);

			glfwSwapBuffers(m_Window);
			glfwPollEvents();

			for (int i = 0; i < (int)m_RenderListeners.size(); ++i)
			{
				m_RenderListeners[i]->RenderOneFrameEnd(dt);
			}

		}

		timer.End();
		dt = timer.DeltaTime(TAT_MAX);

		//TATErrorReporter::Instance()->ReportErr("renderStep: " + TString::ConvertFloat(dt));
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
		{
			actor->Update(dt);
			unit = actor->m_RenderUnit;
		}

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