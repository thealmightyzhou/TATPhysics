#pragma once

#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "../TATCommon/TATCore.h"

using namespace std;

class TATimer
{
public:
	TATimer()
	{
		m_TimeBegin = 0;
		m_TimeEnd = 0;
		m_TimeCollecter = 0;
		m_FirstFrame = true;
	}

	void Begin()
	{
		m_TimeBegin = clock();
	}

	void End()
	{
		m_TimeEnd = clock();
	}

	float DeltaTime(float max)
	{
		float dt = float(m_TimeEnd - m_TimeBegin) / float(CLOCKS_PER_SEC);

		_Clamp(dt, FLT_MIN, max);

		return dt;
	}

	int FramePerSec(float max)
	{
		return static_cast<int>(float(1) / DeltaTime(max));
	}

	//block or not to avoid run too fast
	//keep min deltaTime to eps
	bool Block(float dt, float eps)
	{
		if (m_FirstFrame)
		{
			m_FirstFrame = false;
			return false;
		}

		m_TimeCollecter += dt;

		if (m_TimeCollecter < eps)
			return true;
		else
		{
			m_TimeCollecter = 0;
			return false;
		}
	}

	clock_t m_TimeBegin;
	clock_t m_TimeEnd;
	float m_TimeCollecter;
	bool m_FirstFrame;
};

