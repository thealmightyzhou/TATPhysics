#pragma once

#include "TATMatrix3.h"

class TATransform
{
public:
	TATMatrix3 m_Basis;
	TATVector3 m_Origin;

	TATransform() {}

	TATransform(const TATQuaternion& q, const TATVector3& v) :m_Basis(q), m_Origin(v)
	{}

	TATransform(const TATMatrix3& m, const TATVector3& v) :m_Basis(m), m_Origin(v)
	{}

	TATransform(const TATransform& tr) :m_Basis(tr.m_Basis), m_Origin(tr.m_Origin)
	{}

	inline TATransform& operator=(const TATransform& other)
	{
		m_Basis = other.m_Basis;
		m_Origin = other.m_Origin;
		return *this;
	}

	inline TATVector3 operator()(const TATVector3& x) const
	{
		return x.Dot3(m_Basis[0], m_Basis[1], m_Basis[2]) + m_Origin;
	}

	inline TATVector3 operator*(const TATVector3& x) const
	{
		return (*this)(x);
	}

	TATQuaternion GetRotation() const
	{
		TATQuaternion q;
		m_Basis.GetRotation(q);
		return q;
	}

	TATVector3 GetOrigin() const
	{
		return m_Origin;
	}

	TATMatrix3 GetBasis() const
	{
		return m_Basis;
	}

	inline TATQuaternion operator*(const TATQuaternion& q) const
	{
		return GetRotation() * q;
	}

	inline void SetOrigin(const TATVector3& origin)
	{
		m_Origin = origin;
	}

	inline void SetRotation(const TATQuaternion& q)
	{
		m_Basis.SetRotation(q);
	}

	inline void SetIdentity()
	{
		m_Basis.SetIdentity();
		m_Origin.SetZero();
	}

	TATransform& operator*=(const TATransform& t)
	{
		m_Origin += m_Basis * t.m_Origin;
		m_Basis *= t.m_Basis;
		return *this;
	}

	TATransform Inverse() const
	{
		TATMatrix3 inv = m_Basis.Transpose();
		return TATransform(inv, inv * -m_Origin);
	}

	static const TATransform& GetIdentity()
	{
		static const TATransform identityTransform(TATMatrix3::GetIdentity(),TATVector3::Zero());
		return identityTransform;
	}

	inline TATransform InverseTimes(const TATransform& t) const
	{
		TATVector3 v = t.GetOrigin() - m_Origin;
		return TATransform(m_Basis.TransposeTimes(t.m_Basis), v * m_Basis);
	}

	void GetOpenGLMatrix(float* m) const
	{
		m_Basis.GetOpenGLSubMatrix(m);
		m[12] = m_Origin.X;
		m[13] = m_Origin.Y;
		m[14] = m_Origin.Z;
		m[15] = float(1.0);
	}
};