#pragma once
#include <iostream>
#include <vector>
#include <assert.h>

template<class T>
class TATObjectPool
{
public:
	TATObjectPool(int maxSize):m_MaxSize(maxSize)
	{
		assert(maxSize > 0);
		m_Objects = new T[maxSize];
		m_UsedMap = new bool[maxSize];
		m_NextUnused = 0;
		m_UsedCount = 0;
	}

	~TATObjectPool()
	{
		delete m_Objects;
		delete m_UsedMap;
	}

	T* FetchUnused()
	{
		if (m_UsedCount >= m_MaxSize)
			return 0;

		m_UsedCount++;

		if (!m_UsedMap[m_NextUnused])
		{
			m_UsedMap[m_NextUnused] = true;
			return &m_Objects[m_NextUnused];
		}
		else
		{
			for (int i = 0; i < m_MaxSize; i++)
			{
				if (!m_UsedMap[i])
				{
					m_UsedMap[i] = true;
					return &m_Objects[i];
				}
			}
		}
	}

	void ReturnUsed(T*& obj)
	{
		for (int i = 0; i < m_MaxSize; i++)
		{
			if (&m_Objects[i] == obj)
			{
				m_NextUnused = i;
				m_UsedMap[i] = false;
				m_UsedCount--;
				obj = 0;
				return;
			}
		}

		delete obj;
		obj = 0;
	}

protected:
	T* m_Objects;
	int m_NextUnused;
	bool* m_UsedMap;
	int m_MaxSize;
	int m_UsedCount;
};