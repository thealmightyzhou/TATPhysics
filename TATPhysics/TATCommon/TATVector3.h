#pragma once

#include "TATCore.h"
#include "../TATGLRender/TATGLHeader.h"

#define TAT_MAXVECTOR3 TATVector3(TAT_MAX,TAT_MAX,TAT_MAX)
#define TAT_MINVECTOR3 TATVector3(TAT_EPSILON,TAT_EPSILON,TAT_EPSILON)

class TATVector3
{
public:
	union 
	{
		float m_Datas[3];
		struct
		{
			float X, Y, Z;
		};
	};

	TATVector3(const float& x, const float& y, const float& z) :X(x), Y(y), Z(z)
	{}

	TATVector3(float* bf)
	{
		m_Datas[0] = bf[0];
		m_Datas[1] = bf[1];
		m_Datas[2] = bf[2];
	}

	TATVector3(float x)
	{
		SetValue(x, x, x);
	}

	TATVector3()
	{
		SetZero();
	}

	TATVector3(const TATVector3& v)
	{
		X = v.X;
		Y = v.Y;
		Z = v.Z;
	}

	inline TATVector3 operator+(const TATVector3& v) const
	{
		return TATVector3(X + v.X, Y + v.Y, Z + v.Z);
	}

	inline TATVector3 operator-(const TATVector3& v) const
	{
		return TATVector3(X - v.X, Y - v.Y, Z - v.Z);
	}
	
	inline TATVector3 operator-() const
	{
		return TATVector3(-X, -Y, -Z);
	}

	inline TATVector3 operator*(const float& f) const
	{
		return TATVector3(X * f, Y * f, Z * f);
	}

	inline TATVector3 operator*(const TATVector3& v) const
	{
		return TATVector3(X * v.X, Y * v.Y, Z * v.Z);
	}

	inline TATVector3 operator/(const float& f) const
	{
		assert(f != 0);
		return (*this) * ((1.0f) / f);
	}

	inline TATVector3& operator=(const TATVector3& other)
	{
		SetValue(other.X, other.Y, other.Z);
		return *this;
	}

	inline TATVector3& operator+=(const TATVector3& other)
	{
		X += other.X;
		Y += other.Y;
		Z += other.Z;
		return *this;
	}

	inline TATVector3& operator-=(const TATVector3& other)
	{
		X -= other.X;
		Y -= other.Y;
		Z -= other.Z;
		return *this;
	}

	inline TATVector3& operator*=(const float& f)
	{
		X *= f;
		Y *= f;
		Z *= f;
		return *this;
	}

	inline TATVector3& operator/=(const float& f)
	{
		assert(f != 0.0);

		return *this *= (1.0f / f);
	}

	inline bool operator==(const TATVector3& other)
	{
		return (X == other.X &&
				Y == other.Y &&
				Z == other.Z);
	}

	inline bool operator!=(const TATVector3& other)
	{
		return !(*this == other);
	}

	inline const float& operator[](int index) const
	{
		assert(index >= 0 && index < 3);
		return m_Datas[index];
	}

	inline float Dot(const TATVector3& other) const
	{
		return X * other.X + Y * other.Y + Z * other.Z;
	}

	inline TATVector3 Cross(const TATVector3& other) const
	{
		return TATVector3(
			Y * other.Z - Z * other.Y,
			Z * other.X - X * other.Z,
			X * other.Y - Y * other.X);
	}
	
	inline float Length2() const
	{
		return Dot(*this);
	}

	inline float Length() const
	{
		return sqrt(Length2());
	}

	inline float Distance(const TATVector3& other)
	{
		return TATVector3(X - other.X, Y - other.Y, Z - other.Z).Length();
	}

	inline float Distance2(const TATVector3& other)
	{
		return TATVector3(X - other.X, Y - other.Y, Z - other.Z).Length2();
	}

	inline TATVector3& SafeNormalize()
	{
		float len2 = Length2();

		if (len2 >= TAT_EPSILON * TAT_EPSILON)
		{
			(*this) /= sqrt(len2);
		}
		else
		{
			SetValue(1, 0, 0);
		}
		return *this;
	}

	inline TATVector3 Normalized() const
	{
		return *this / Length();
	}

	inline void SetValue(const float& x, const float& y, const float& z)
	{
		X = x;
		Y = y;
		Z = z;
	}

	inline void SetMax(const TATVector3& other)
	{
		_SetMax(X, other.X);
		_SetMax(Y, other.Y);
		_SetMax(Z, other.Z);
	}

	inline void SetMin(const TATVector3& other)
	{
		_SetMin(X, other.X);
		_SetMin(Y, other.Y);
		_SetMin(Z, other.Z);
	}

	inline static TATVector3 MakeMax(const TATVector3& v1, const TATVector3& v2)
	{
		TATVector3 res = -TAT_MAXVECTOR3;
		res.SetMax(v1);
		res.SetMax(v2);
		return res;
	}

	inline static TATVector3 MakeMin(const TATVector3& v1, const TATVector3& v2)
	{
		TATVector3 res = TAT_MAXVECTOR3;
		res.SetMin(v1);
		res.SetMin(v2);
		return res;
	}

	//return cross matrix
	inline void GetSkewSymmetricMatrix(TATVector3* v0, TATVector3* v1, TATVector3* v2)
	{
		v0->SetValue(0, -Z, Y);
		v1->SetValue(Z, 0, -X);
		v2->SetValue(-Y, X, 0);
	}

	inline TATVector3 Dot3(const TATVector3& v0, const TATVector3& v1, const TATVector3& v2) const
	{
		return TATVector3(Dot(v0), Dot(v1), Dot(v2));
	}

	inline void SetZero()
	{
		SetValue(float(0.), float(0.), float(0.));
	}

	inline static TATVector3 UnitX()
	{
		static const TATVector3 unitXVector(1, 0, 0);
		return unitXVector;
	}

	inline static TATVector3 UnitY()
	{
		static const TATVector3 unitYVector(0, 1, 0);
		return unitYVector;
	}

	inline static TATVector3 UnitZ()
	{
		static const TATVector3 unitZVector(0, 0, 1);
		return unitZVector;
	}

	inline static TATVector3 Zero()
	{
		static const TATVector3 zeroVector(0, 0, 0);
		return zeroVector;
	}

	inline static TATVector3 One()
	{
		static const TATVector3 oneVector(1, 1, 1);
		return oneVector;
	}

	bool IsZero()
	{
		return *this == Zero();
	}

	glm::vec3 ToGLM()
	{
		return glm::vec3(X, Y, Z);
	}
};

inline TATVector3 operator*(const float& s, const TATVector3& v)
{
	return v * s;
}