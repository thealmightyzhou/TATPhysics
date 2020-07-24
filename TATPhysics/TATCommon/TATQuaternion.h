#pragma once

#include "TATVector3.h"

class TATQuaternion
{
public:
	TATQuaternion(const float& x, const float& y, const float& z, const float& w)
	{
		SetValue(x, y, z, w);
	}
	TATQuaternion(const TATQuaternion& q)
	{
		SetValue(q.X, q.Y, q.Z, q.W);
	}
	TATQuaternion(const TATVector3& v, const float& w)
	{
		SetValue(v.X, v.Y, v.Z, w);
	}
	TATQuaternion()
	{
		SetIdentity();
	}
	union
	{
		float m_Datas[4];
		struct
		{
			float X, Y, Z, W;
		};
	};

	inline void SetValue(const float& x, const float& y, const float& z, const float& w)
	{
		X = x;
		Y = y;
		Z = z;
		W = w;
	}

	void FromAngleAxis(const TATVector3& axis1, const float& angle)
	{
		TATVector3 axis = axis1;
		axis.SafeNormalize();

		float d = axis.Length();
		assert(d != float(0.0));
		if (d < TAT_EPSILON)
		{
			SetValue(0, 0, 0, 1);
		}
		else
		{
			float s = sin(angle * float(0.5)) / d;
			SetValue(axis.X * s, axis.Y * s, axis.Z * s, cos(angle * float(0.5)));
		}
	}

	void FromEuler(const float& yaw, const float& pitch, const float& roll)
	{
		float halfYaw = float(yaw) * float(0.5);
		float halfPitch = float(pitch) * float(0.5);
		float halfRoll = float(roll) * float(0.5);
		float cosYaw = cos(halfYaw);
		float sinYaw = sin(halfYaw);
		float cosPitch = cos(halfPitch);
		float sinPitch = sin(halfPitch);
		float cosRoll = cos(halfRoll);
		float sinRoll = sin(halfRoll);
		SetValue(cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,
				 cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,
				 sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,
				 cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);
	}

	TATQuaternion& operator*=(const float& s)
	{
		m_Datas[0] *= s;
		m_Datas[1] *= s;
		m_Datas[2] *= s;
		m_Datas[3] *= s;

		return *this;
	}

	TATQuaternion& operator/=(const float& s)
	{
		assert(s != float(0.0));
		return *this *= float(1.0) / s;
	}

	TATQuaternion& operator*=(const TATQuaternion& q)
	{
		SetValue(
			m_Datas[3] * q.X + m_Datas[0] * q.m_Datas[3] + m_Datas[1] * q.Z - m_Datas[2] * q.Y,
			m_Datas[3] * q.Y + m_Datas[1] * q.m_Datas[3] + m_Datas[2] * q.X - m_Datas[0] * q.Z,
			m_Datas[3] * q.Z + m_Datas[2] * q.m_Datas[3] + m_Datas[0] * q.Y - m_Datas[1] * q.X,
			m_Datas[3] * q.m_Datas[3] - m_Datas[0] * q.X - m_Datas[1] * q.Y - m_Datas[2] * q.Z);

		return *this;
	}

	inline TATQuaternion operator*(const float& s) const
	{
		return TATQuaternion(X * s, Y * s, Z * s, m_Datas[3] * s);
	}

	TATQuaternion operator/(const float& s) const
	{
		assert(s != float(0.0));
		return *this * (float(1.0) / s);
	}

	float Dot(const TATQuaternion& q) const
	{
		return m_Datas[0] * q.X +
			   m_Datas[1] * q.Y +
			   m_Datas[2] * q.Z +
			   m_Datas[3] * q.m_Datas[3];
	}

	float Length2() const
	{
		return Dot(*this);
	}

	float Length() const
	{
		return sqrt(Length2());
	}

	TATQuaternion& Normalize()
	{
		return *this /= Length();
	}

	TATQuaternion Normalized() const
	{
		return *this / Length();
	}

	float Angle(const TATQuaternion& q) const
	{
		float s = sqrt(Length2() * q.Length2());
		assert(s != float(0.0));
		return acosf(Dot(q) / s);
	}

	float GetAngle() const
	{
		float s = float(2.) * acosf(m_Datas[3]);
		return s;
	}

	TATVector3 GetAxis() const
	{
		float s_squared = 1.f - m_Datas[3] * m_Datas[3];

		if (s_squared < float(10.) * TAT_EPSILON)  //Check for divide by zero
			return TATVector3(1.0, 0.0, 0.0);     // Arbitrary
		float s = 1.f / sqrt(s_squared);
		return TATVector3(m_Datas[0] * s, m_Datas[1] * s, m_Datas[2] * s);
	}

	TATQuaternion Inverse() const
	{
		return TATQuaternion(-m_Datas[0], -m_Datas[1], -m_Datas[2], m_Datas[3]);
	}

	inline TATQuaternion operator+(const TATQuaternion& q2) const
	{
		return TATQuaternion(X + q2.X, Y + q2.Y, Z + q2.Z, m_Datas[3] + q2.m_Datas[3]);
	}

	inline TATQuaternion operator-(const TATQuaternion& q2) const
	{
		return TATQuaternion(X - q2.X, Y - q2.Y, Z - q2.Z, m_Datas[3] - q2.m_Datas[3]);
	}

	inline TATQuaternion operator-() const
	{
		return TATQuaternion(-X, -Y, -Z, -W);
	}

	static const TATQuaternion& GetIdentity()
	{
		static const TATQuaternion identityQuat(float(0.), float(0.), float(0.), float(1.));
		return identityQuat;
	}

	inline void SetIdentity()
	{
		SetValue(float(0.), float(0.), float(0.), float(1.));
	}

	inline TATQuaternion operator*(const TATQuaternion& q)
	{
		return TATQuaternion(
			W * q.X + X * q.W + Y * q.Z - Z * q.Y,
			W * q.Y + Y * q.W + Z * q.X - X * q.Z,
			W * q.Z + Z * q.W + X * q.Y - Y * q.X,
			W * q.W - X * q.X - Y * q.Y - Z * q.Z);
	}

	inline TATVector3 operator*(const TATVector3& v)
	{
		TATQuaternion q = Multi(v);
		q *= this->Inverse();

		return TATVector3(q.X, q.Y, q.Z);
	}

	inline TATQuaternion Multi(const TATVector3& w)
	{
		return TATQuaternion(
			+w.X * W + w.Y * Z - w.Z * Y,
			+w.Y * W + w.Z * X - w.X * Z,
			+w.Z * W + w.X * Y - w.Y * X,
			-w.X * X - w.Y * Y - w.Z * Z);
	}
};