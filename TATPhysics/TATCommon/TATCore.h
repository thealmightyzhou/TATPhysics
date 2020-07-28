#pragma once
#include <iostream>
#include <assert.h>
#include <stdlib.h>
#include <vector>

typedef unsigned int UINT;

#define TAT_EPSILON			FLT_EPSILON
#define TAT_EPSILON2		1e-6
#define TAT_MAX				FLT_MAX
#define TAT_MAX2			1e10	
#define TAT_RAND_MAX		RAND_MAX
#define TAT_SHADERID_UNUSE	0
#define MAX_TEXCOORDINATE_COUNT 5

#define TAT_2PI float(6.283185307179586232)
#define TAT_PI (TAT_2PI * float(0.5))
#define TAT_HALF_PI (TAT_2PI * float(0.25))

#define TAT_REGISTER_GET(TYPE,ATTR)							\
	inline const TYPE& Get##ATTR() const {return m_##ATTR;}

#define TAT_REGISTER_SET(TYPE,ATTR)							\
	inline void Set##ATTR(const TYPE& x) {m_##ATTR = x;}

#define TAT_REGISTER_GETSET(TYPE,ATTR)						\
	TAT_REGISTER_GET(TYPE,ATTR)								\
	TAT_REGISTER_SET(TYPE,ATTR)		

#define TAT_REGISTER_ATTRIBUTE(TYPE,ATTR)					\
				TYPE m_##ATTR;								\
	TAT_REGISTER_GETSET(TYPE,ATTR)	

#define TAT_REGISTER_ATTRIBUTE_GET(TYPE,ATTR)				\
	TYPE m_##ATTR;											\
	TAT_REGISTER_GET(TYPE,ATTR)

#define	TAT_MEMSET(arr,val)	memset(arr,val,sizeof(arr));

#define TAT_MEMCPY(arr0,arr1) memcpy(arr0,arr1,sizeof(arr1));

#define TAT_ARRAY_SIZE(arr) ((int)(sizeof(arr) / sizeof(arr[0])))

#define TAT_SAFE_NEW(arr,type,size)				\
	if(size == 0)								\
	{											\
		if(arr)									\
			delete arr;							\
		arr = 0;								\
	}											\
	else if(!arr)								\
		arr = new type[size];					\
	else if(arr && TAT_ARRAY_SIZE(arr) != size)	\
	{											\
		delete arr;								\
		arr = new type[size];					\
	}											\

#define TAT_MEMEMPTY(ARR,ZERO_DETERMINE,RESULT)	\
{												\
	size_t size = sizeof(ARR) / sizeof(ARR[0]); \
	RESULT = true;								\
	for (int i = 0; i < (int)size; ++i)			\
	{											\
		if (ARR[i] != ZERO_DETERMINE)			\
			RESULT = false;						\
	}											\
}

#define TAT_ARRAYCONTAIN(ARR1,ARR2,NUM1,NUM2,RES)	\
{													\
	RES = true;										\
	for (int i = 0; i < NUM1; i++)					\
	{												\
		bool hasEle = false;						\
		for (int j = 0; j < NUM2; j++)				\
		{											\
			if (ARR2[j] == ARR1[i])					\
			{										\
				hasEle = true;						\
			}										\
		}											\
		if (!hasEle)								\
		{											\
			RES = false;							\
			break;									\
		}											\
	}												\
}											

#define TATResourcePtr std::shared_ptr 

#define TAT_POOL_OBJECT(classname)	\
		classname();				\
		void Clear();				\

inline void _SetMax(float& x, const float& y)
{
	x = x > y ? x : y;
}

inline void _SetMin(float& x, const float& y)
{
	x = x < y ? x : y;
}

inline float _RandRange(float min, float max)
{
	return (rand() / (float(TAT_RAND_MAX) + float(1.0))) * (max - min) + min;
}

template<class T>
inline void _Clamp(T& val, T min, T max)
{
	if (max < min)
		std::swap(min, max);

	if (val < min)
		val = min;
	if (max < val)
		val = max;
}