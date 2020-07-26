#pragma once
#include <iostream>

template<class T>
class TATResourcePtr
{
public:
	TATResourcePtr(T* res)
	{
		m_SharedPtr = std::shared_ptr<T>(res);
	}

	std::shared_ptr<T> m_SharedPtr;
};