#pragma once
#include "../TATCommon/TATCore.h"
#include "../TATCommon/TATVector3.h"
#include <queue>

typedef void (*TATRenderTask)();
typedef void (*TATDrawLineTask)(const TATVector3& p0, const TATVector3& p1, const TATVector3& col);

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

	std::queue<TATRenderTask> m_TaskQueue;

	std::queue<TATaskDataDrawLine> m_DrawLineTaskQueue;
};