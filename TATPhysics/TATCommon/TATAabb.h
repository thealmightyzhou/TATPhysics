#pragma once
#include "TATVector3.h"
#include "TATransform.h"

struct TATAabb
{
public:
	TATAabb()
	{
		m_OriginMin = TAT_MAXVECTOR3;
		m_OriginMax = -TAT_MAXVECTOR3;
		m_Min = m_OriginMin;
		m_Max = m_OriginMax;
	}

	TATAabb(const TATVector3& min, const TATVector3& max) :m_OriginMin(min), m_OriginMax(max), m_Min(min), m_Max(max)
	{}

	//will auto compute aabb min max
	void SetOrigin(const TATVector3& min, const TATVector3& max)
	{
		TATVector3 mi = min;
		mi.SetMin(max);
		TATVector3 mx = max;
		mx.SetMax(min);
		m_Min = m_OriginMin = mi;
		m_Max = m_OriginMax = mx;
	}

	void SetOriginExtend(const TATVector3& center, const TATVector3& extend)
	{
		m_Min = m_OriginMin = center - extend;
		m_Max = m_OriginMax = center + extend;
	}

	void SetOriginSphere(const TATVector3& center, float r)
	{
		m_Min = m_OriginMin = center - r * TATVector3::One();
		m_Max = m_OriginMax = center + r * TATVector3::One();
	}

	void GetWorldBoundingSphere(TATVector3& center, float& radius)
	{
		center = (m_Min + m_Max) / 2;
		radius = m_Min.Distance(m_Max) / 2;
	}

	void GetWorldBoundingAabb(TATVector3& min, TATVector3& max)
	{
		min = m_Min;
		max = m_Max;
	}

	void GetWorldBoundingBox(TATVector3& center, TATVector3& extend)
	{
		center = (m_Min + m_Max) / 2;
		extend = (m_Max - m_Min) / 2;
	}

	void GetLocalBoundingSphere(TATVector3& center, float& radius)
	{
		center = (m_OriginMin + m_OriginMax) / 2;
		radius = m_OriginMin.Distance(m_OriginMax) / 2;
	}

	void GetLocalBoundingAabb(TATVector3& min, TATVector3& max)
	{
		min = m_OriginMin;
		max = m_OriginMax;
	}

	void GetLocalBoundingBox(TATVector3& center, TATVector3& extend)
	{
		center = (m_OriginMin + m_OriginMax) / 2;
		extend = (m_OriginMax - m_OriginMin) / 2;
	}

	void Update(const TATransform& tr)
	{
		float x0 = m_OriginMin.X;
		float y0 = m_OriginMin.Y;
		float z0 = m_OriginMin.Z;
		float x1 = m_OriginMax.X;
		float y1 = m_OriginMax.Y;
		float z1 = m_OriginMax.Z;

		TATVector3 pts[8]{ m_OriginMin ,TATVector3(x1,y0,z0) ,TATVector3(x1,y0,z1) ,TATVector3(x0,y0,z1),
						  TATVector3(x0,y1,z0) ,TATVector3(x1,y1,z0) ,m_OriginMax ,TATVector3(x0,y1,z1) };

		m_Min = TAT_MAXVECTOR3;
		m_Max = -TAT_MAXVECTOR3;

		for (int i = 0; i < 8; i++)
		{
			pts[i] = tr * pts[i];
			m_Min.SetMin(pts[i]);
			m_Max.SetMax(pts[i]);
		}
	}

	TATVector3 m_Min;
	TATVector3 m_Max;

	TATVector3 m_OriginMin;
	TATVector3 m_OriginMax;
};