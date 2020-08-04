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
		SetValue(sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw,   //x
			cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw,   //y
			cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw,   //z
			cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw);  //formerly yzx

		Normalize();
	}

	void FromCoordinateSys(const TATVector3& x, const TATVector3& y, const TATVector3& z)
	{
		TATQuaternion orient = FastestRotation(TATVector3::UnitX(), x);
		TATVector3 dz = orient * z;
		orient *= FastestRotation(dz, z);
		SetValue(orient.X, orient.Y, orient.Z, orient.W);
	}

	void FromTwoCoordinateSys(const TATVector3& fromX, const TATVector3& fromY, const TATVector3& fromZ,
		const TATVector3& toX, const TATVector3& toY, const TATVector3& toZ)
	{
		TATQuaternion orient = FastestRotation(fromZ, toZ);
		TATVector3 dy = orient * fromY;
		orient *= FastestRotation(dy, toY);
		SetValue(orient.X, orient.Y, orient.Z, orient.W);
	}

	static TATQuaternion FastestRotation(const TATVector3& v1, const TATVector3& v2)
	{
		TATVector3 axis0 = v1.Normalized();
		TATVector3 axis1 = v2.Normalized();

		TATVector3 rotAxis = axis0.Cross(axis1);
		float angle = acos(axis0.Dot(axis1));
		TATQuaternion rot;
		rot.FromAngleAxis(rotAxis, angle);

		return rot;
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
			+w.X * W + w.Z * Y - w.Y * Z,
			+w.Y * W + w.X * Z - w.Z * X,
			+w.Z * W + w.Y * X - w.X * Y,
			-w.X * X - w.Y * Y - w.Z * Z);
	}

	void GetEulerZYX(float& yawZ, float& pitchY, float& rollX) const
	{
		float squ;
		float sqx;
		float sqy;
		float sqz;
		float sarg;
		sqx = m_Datas[0] * m_Datas[0];
		sqy = m_Datas[1] * m_Datas[1];
		sqz = m_Datas[2] * m_Datas[2];
		squ = m_Datas[3] * m_Datas[3];
		rollX = atan2(2 * (m_Datas[1] * m_Datas[2] + m_Datas[3] * m_Datas[0]), squ - sqx - sqy + sqz);
		sarg = float(-2.) * (m_Datas[0] * m_Datas[2] - m_Datas[3] * m_Datas[1]);
		pitchY = sarg <= float(-1.0) ? float(-0.5) * TAT_PI : (sarg >= float(1.0) ? float(0.5) * TAT_PI : asin(sarg));
		yawZ = atan2(2 * (m_Datas[0] * m_Datas[1] + m_Datas[3] * m_Datas[2]), squ + sqx - sqy - sqz);
	}
};