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

	bool inline Penetration(const TATRange& other, void*& data0, void*& data1, float& dist)
	{
		if (other.m_MinRange < m_MinRange && m_MaxRange < other.m_MaxRange)
		{
			data0 = m_MinData;
			data1 = m_MaxData;
			dist = m_MaxRange - m_MinRange;
			return false;
		}
		else if (m_MinRange < other.m_MinRange && other.m_MaxRange < m_MaxRange)
		{
			data0 = other.m_MinData;
			data1 = other.m_MaxData;
			dist = other.m_MaxRange - other.m_MinRange;
			return false;
		}
		else if (m_MaxRange < other.m_MinRange)
		{
			data0 = m_MaxData;
			data1 = other.m_MinData;
			dist = m_MaxRange - other.m_MinRange;
			return false;

		}
		else if (m_MinRange > other.m_MaxRange)
		{
			data0 = m_MinData;
			data1 = other.m_MaxData;
			dist = other.m_MaxRange - m_MinRange;
			return false;
		}

		float val1 = m_MaxRange - other.m_MinRange;
		float val2 = other.m_MaxRange - m_MinRange;

		if (val1 < val2)
		{
			data0 = m_MaxData;
			data1 = other.m_MinData;
			dist = val1;
			return true;
		}
		else
		{
			data0 = m_MinData;
			data1 = other.m_MaxData;
			dist = val2;
			return true;
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

	float Penetration(const TATRange& other)
	{
		if (other.m_MaxRange > m_MaxRange && other.m_MinRange < m_MinRange)
		{
			return m_MaxRange - m_MinRange;
		}
		if (m_MaxRange > other.m_MaxRange && m_MinRange < other.m_MinRange)
		{
			return other.m_MaxRange - other.m_MinRange;
		}

		float val1 = m_MaxRange - other.m_MinRange;
		float val2 = other.m_MaxRange - m_MinRange;

		if (val1 < val2)
		{
			return val1;
		}
		else
		{
			return val2;
		}
	}

	//seperate or penetration
	//@dist = seperate: + value ; penetrate: - value
	//------      1.seperate
	//        ---
	//------	  2.include
	//  ---
	//-----       3.penetration
	//   -----    
	bool Distance(const TATRange& other, float& dist, int item[2])
	{
		if (m_MaxRange <= other.m_MinRange)
		{
			dist = other.m_MinRange - m_MaxRange;
			item[0] = 1;
			item[1] = 2;
			return true;
		}
		else if (other.m_MaxRange <= m_MinRange)
		{
			dist = m_MinRange - other.m_MaxRange;
			item[0] = 0;
			item[1] = 3;
			return true;
		}
		else if (m_MinRange <= other.m_MinRange && other.m_MaxRange <= m_MaxRange)
		{
			float pen0 = other.m_MaxRange - m_MinRange;
			float pen1 = m_MaxRange - other.m_MinRange;
			bool side = pen0 < pen1;
			dist = side ? -pen0 : -pen1;
			item[0] = side ? 0 : 1;
			item[1] = side ? 3 : 2;
			return false;
		}
		else if (other.m_MinRange <= m_MinRange && m_MaxRange <= other.m_MaxRange)
		{
			float pen0 = m_MaxRange - other.m_MinRange;
			float pen1 = other.m_MaxRange - m_MinRange;
			bool side = pen0 < pen1;
			dist = side ? -pen0 : -pen1;
			item[0] = side ? 1 : 0;
			item[1] = side ? 2 : 3;
			return false;
		}
		else if (m_MinRange < other.m_MinRange)
		{
			dist = other.m_MinRange - m_MaxRange;
			item[0] = 1;
			item[1] = 2;
			return false;
		}
		else if (other.m_MinRange < m_MinRange)
		{
			dist = m_MinRange - other.m_MaxRange;
			item[0] = 0;
			item[1] = 3;
			return false;
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