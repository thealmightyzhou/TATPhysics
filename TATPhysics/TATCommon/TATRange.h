#pragma once
#include "TATVector3.h"

struct TATRange
{
public:
	TATRange(float min, float max) :m_MinRange(min), m_MaxRange(max)
	{
		if (m_MinRange > m_MaxRange)
			std::swap(m_MinRange, m_MaxRange);
	}

	bool inline Coincide(const TATRange& other)
	{
		if (m_MaxRange < other.m_MinRange || m_MinRange>other.m_MaxRange)
			return false;
		else
			return true;
	}

	float inline Penetration(const TATRange& other, void*& data0, void*& data1)
	{
		if (other.m_MaxRange > m_MaxRange&& other.m_MinRange < m_MinRange)
		{
			data0 = m_MinData;
			data1 = m_MaxData;
			return m_MaxRange - m_MinRange;
		}
		else if (m_MaxRange > other.m_MaxRange&& m_MinRange < other.m_MinRange)
		{
			data0 = other.m_MinData;
			data1 = other.m_MaxData;
			return other.m_MaxRange - other.m_MinRange;
		}
		else if (m_MaxRange < other.m_MinRange)
		{
			data0 = m_MaxData;
			data1 = other.m_MinData;
			return m_MaxRange - other.m_MinRange;
		}
		else if (m_MinRange > other.m_MaxRange)
		{
			data0 = m_MinData;
			data1 = other.m_MaxData;
			return other.m_MaxRange - m_MinRange;
		}

		float val1 = m_MaxRange - other.m_MinRange;
		float val2 = other.m_MaxRange - m_MinRange;

		if (val1 < val2)
		{
			data0 = other.m_MinData;
			data1 = m_MaxData;
			return val1;
		}
		else
		{
			data0 = m_MinData;
			data1 = other.m_MaxData;
			return val2;
		}

	}

	//only return this->data
	float Penetration(const TATRange& other, void*& data)
	{
		if (other.m_MaxRange > m_MaxRange&& other.m_MinRange < m_MinRange)
		{
			data = 0;
			return m_MaxRange - m_MinRange;
		}
		if (m_MaxRange > other.m_MaxRange&& m_MinRange < other.m_MinRange)
		{
			data = 0;
			return other.m_MaxRange - other.m_MinRange;
		}

		float val1 = m_MaxRange - other.m_MinRange;
		float val2 = other.m_MaxRange - m_MinRange;

		if (val1 < val2)
		{
			data = m_MaxData;
			return val1;
		}
		else
		{
			data = m_MinData;
			return val2;
		}
	}

	void SetUserData(void* minData, void* maxData)
	{
		m_MinData = minData;
		m_MaxData = maxData;
	}

	float m_MinRange, m_MaxRange;
	void* m_MinData;
	void* m_MaxData;

};