#pragma once
#include "../TATCommon/TATCore.h"
#include "../TATCommon/TATVector3.h"
#include "../TATStage/TATActor.h"
#include <queue>

typedef void (*TATRenderTask)();
typedef void (*TATDrawLineTask)(const TATVector3& p0, const TATVector3& p1, const TATVector3& col);
typedef void (*TATDrawSoftBodyTask)(std::vector<TATVector3> points, TATActor* actor);

struct TATaskDataDrawLine
{
public:
	TATaskDataDrawLine(const TATVector3& p0, const TATVector3& p1, const TATVector3& col, TATDrawLineTask task)
		:m_Point0(p0), m_Point1(p1), m_Color(col), m_Task(task)
	{}

	TATVector3 m_Point0, m_Point1;
	TATVector3 m_Color;

	TATDrawLineTask m_Task;
};

struct TATaskDataDrawSoftBody
{
public:
	TATaskDataDrawSoftBody(std::vector<TATVector3>& points, TATActor* actor, TATDrawSoftBodyTask task) :
		m_Actor(actor), m_Task(task)
	{
		m_RenderPoints.assign(points.begin(), points.end());
	}

	std::vector<TATVector3> m_RenderPoints;
	TATActor* m_Actor;
	TATDrawSoftBodyTask m_Task;

};

class TATRenderTaskList
{
public:
	void PushTask(TATRenderTask task)
	{
		m_TaskQueue.push(task);
	}

	//draw line task
	void PushTask(TATDrawLineTask task, const TATVector3& p0, const TATVector3& p1, const TATVector3& col)
	{
		m_DrawLineTaskQueue.push(TATaskDataDrawLine(p0, p1, col, task));
	}

	//update softbody task
	void PushTask(TATDrawSoftBodyTask task, std::vector<TATVector3>& points, TATActor* actor)
	{
		m_DrawSoftBodyQueue.push(TATaskDataDrawSoftBody(points, actor, task));
	}

	void HandleTask()
	{
		int sz = m_TaskQueue.size();
		for (int i = 0; i < sz; ++i)
		{
			TATRenderTask task = m_TaskQueue.front();
			m_TaskQueue.pop();
			task();
		}

		HandleDrawLines();

		HandleDrawSoftBodys();
	}

	void HandleDrawLines()
	{
		int sz = m_DrawLineTaskQueue.size();
		for (int i = 0; i < sz; ++i)
		{
			TATaskDataDrawLine& data = m_DrawLineTaskQueue.front();
			data.m_Task(data.m_Point0, data.m_Point1, data.m_Color);

			m_DrawLineTaskQueue.pop();
		}
	}

	void HandleDrawSoftBodys()
	{
		int sz = m_DrawSoftBodyQueue.size();
		for (int i = sz - 1; 0 <= i && i < sz; ++i)
		{
			TATaskDataDrawSoftBody& data = m_DrawSoftBodyQueue.front();
			data.m_Task(data.m_RenderPoints, data.m_Actor);

			m_DrawSoftBodyQueue.pop();
		}
		
		while (!m_DrawSoftBodyQueue.empty())
			m_DrawSoftBodyQueue.pop();
	}

	std::queue<TATRenderTask> m_TaskQueue;

	std::queue<TATaskDataDrawLine> m_DrawLineTaskQueue;

	std::queue<TATaskDataDrawSoftBody> m_DrawSoftBodyQueue;
};