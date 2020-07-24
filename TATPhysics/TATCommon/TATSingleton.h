#ifndef THEALMIGHTY_SINGLETON
#define THEALMIGHTY_SINGLETON

#include <iostream>

//template<class T>
//class Singleton
//{
//public:
//	static T* Instance()
//	{
//		if (!m_obj)
//			m_obj = new T;
//		return m_obj;
//	}
//
//protected:
//	static T* m_obj;
//};
//
//template<class T>
//T* Singleton<T>::m_obj = NULL;

template<class T>
class Singleton
{
public:
	static T* Instance()
	{
		return m_obj;
	}
protected:
	static T* m_obj;
};
template<class T>
T* Singleton<T>::m_obj = new T;

#endif // !THEALMIGHTY_SINGLETON