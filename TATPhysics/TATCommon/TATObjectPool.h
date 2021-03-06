#pragma once
#include <iostream>
#include <vector>
#include <assert.h>

template<class T>
//object in pool mush have a Clear() function
class TATObjectPool
{
public:
	TATObjectPool(int maxSize):m_MaxSize(maxSize)
	{
		assert(maxSize > 0);
		m_Objects = new T[maxSize];
		m_UsedMap = new bool[maxSize];
		TAT_MEMSET_NEW(m_UsedMap, maxSize, false);
		m_NextUnused = 0;
		m_UsedCount = 0;
	}

	TATObjectPool() {}

	void Initialize(int max_sz)
	{
		assert(max_sz > 0);
		m_Objects = new T[max_sz];
		m_UsedMap = new bool[max_sz];
		TAT_MEMSET_NEW(m_UsedMap, max_sz, false);
		m_NextUnused = 0;
		m_UsedCount = 0;
	}

	~TATObjectPool()
	{
		delete m_Objects;
		delete m_UsedMap;
		m_UsedObjects.clear();
	}

	T* FetchUnused()
	{
		int index = -1;
		if (m_UsedCount >= m_MaxSize)
			return 0;

		if (!m_UsedMap[m_NextUnused])
		{
			index = m_NextUnused;
		}
		else
		{
			for (int i = 0; i < m_MaxSize; i++)
			{
				if (!m_UsedMap[i])
				{
					index = i;
				}
			}
		}

		if (index < m_MaxSize && 0 <= index)
		{
			m_UsedMap[index] = true;
			m_NextUnused++;
			_Clamp(m_NextUnused, 0, m_MaxSize - 1);
			m_Objects[index].Clear();

			m_UsedCount++;
			m_Objects[index].m_IndexInPool = index;
			m_UsedObjects.push_back(&m_Objects[index]);
			return &m_Objects[index];
		}

		return 0;
	}

	void ReturnUsed(T*& obj)
	{
		typename std::vector<T*>::iterator it = m_UsedObjects.begin();

		while (it != m_UsedObjects.end())
		{
			if((*it) == obj)
				it = m_UsedObjects.erase(it);
			else
				it++;
		}

		int index = obj->m_IndexInPool;
		if (&m_Objects[index] == obj)
		{
			m_NextUnused = index;
			m_UsedMap[index] = false;
			m_UsedCount--;
			obj = 0;
			return;
		}

		delete obj;
		obj = 0;
	}

	void ReturnUsed(int index)
	{
		typename std::vector<T*>::iterator it = m_UsedObjects.begin();
		for (it; it != m_UsedObjects.end(); it++)
		{
			if (*it == &m_Objects[index])
				m_UsedObjects.erase(it);
		}

		m_NextUnused = index;
		m_UsedMap[index] = false;
		m_UsedCount--;
		m_Objects[index].Clear();
		return;
	}

	void Clear()
	{
		for (int i = 0; i < m_UsedObjects.size(); ++i)
		{
			m_UsedObjects[i]->Clear();
		}
		m_UsedObjects.clear();
		m_UsedCount = 0;
		TAT_MEMSET_NEW(m_UsedMap, m_MaxSize, false);
		m_NextUnused = 0;
	}

	const std::vector<T*>& FetchAllUsed()
	{
		return m_UsedObjects;
	}

	T* GetPool()
	{
		return m_Objects;
	}

	T& operator[](int index)
	{
		return m_Objects[index];
	}

protected:
	T* m_Objects;
	int m_NextUnused;
	bool* m_UsedMap;
	int m_MaxSize;
	int m_UsedCount;
	std::vector<T*> m_UsedObjects;
};