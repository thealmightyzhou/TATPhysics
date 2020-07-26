#pragma once

#include "TATQuaternion.h"

class TATMatrix3
{
public:
	TATVector3 Ele[3];

	TATMatrix3() {}

	TATMatrix3(const float& x00, const float& x01, const float& x02,
		const float& x10, const float& x11, const float& x12,
		const float& x20, const float& x21, const float& x22)
	{
		SetValue(x00, x01, x02, x10, x11, x12, x20, x21, x22);
	}

	TATMatrix3(const TATVector3& v0, const TATVector3& v1, const TATVector3& v2)
	{
		Ele[0] = v0;
		Ele[1] = v1;
		Ele[2] = v2;
	}

	TATMatrix3(const TATMatrix3& m)
	{
		Ele[0] = m.Ele[0];
		Ele[1] = m.Ele[1];
		Ele[2] = m.Ele[2];
	}

	TATMatrix3(const TATQuaternion& q)
	{
		SetRotation(q);
	}

	inline void SetValue(const float& x00, const float& x01, const float& x02,
		const float& x10, const float& x11, const float& x12,
		const float& x20, const float& x21, const float& x22)
	{
		Ele[0].SetValue(x00, x01, x02);
		Ele[1].SetValue(x10, x11, x12);
		Ele[2].SetValue(x20, x21, x22);
	}

	inline void SetValue(const TATVector3& v0, const TATVector3& v1, const TATVector3& v2)
	{
		Ele[0] = v0;
		Ele[1] = v1;
		Ele[2] = v2;
	}

	inline float& operator()(int r, int c)
	{
		assert(r >= 0 && r < 3);
		assert(c >= 0 && c < 3);
		return Ele[r].m_Datas[c];
	}

	inline const TATVector3& operator[](int index) const
	{
		assert(index >= 0 && index < 3);
		return Ele[index];
	}

	inline TATMatrix3& operator=(const TATMatrix3& m)
	{
		SetValue(m.Ele[0], m.Ele[1], m.Ele[2]);
		return *this;
	}

	inline TATVector3 Col(int c) const
	{
		assert(c >= 0 && c < 3);
		return TATVector3(Ele[0][c], Ele[1][c], Ele[2][c]);
	}

	inline TATVector3 Row(int r)
	{
		assert(r >= 0 && r < 3);
		return Ele[r];
	}

	inline void SetIdentity()
	{
		SetValue(1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0,
			0.0f, 0.0f, 1.0f);
	}

	inline TATMatrix3& operator*=(const float& f)
	{
		Ele[0] *= f;
		Ele[1] *= f;
		Ele[2] *= f;

		return *this;
	}

	inline TATMatrix3& operator*=(const TATMatrix3& m)
	{
		SetValue(Row(0).Dot(m.Col(0)), Row(0).Dot(m.Col(1)), Row(0).Dot(m.Col(2)),
			Row(1).Dot(m.Col(0)), Row(1).Dot(m.Col(1)), Row(1).Dot(m.Col(2)),
			Row(2).Dot(m.Col(0)), Row(2).Dot(m.Col(1)), Row(2).Dot(m.Col(2)));

		return *this;
	}

	inline TATMatrix3 operator*(const TATMatrix3& m)
	{
		return TATMatrix3(Row(0).Dot(m.Col(0)), Row(0).Dot(m.Col(1)), Row(0).Dot(m.Col(2)),
			Row(1).Dot(m.Col(0)), Row(1).Dot(m.Col(1)), Row(1).Dot(m.Col(2)),
			Row(2).Dot(m.Col(0)), Row(2).Dot(m.Col(1)), Row(2).Dot(m.Col(2)));
	}

	inline TATVector3 operator*(const TATVector3& v) const
	{
		return TATVector3(Ele[0].Dot(v), Ele[1].Dot(v), Ele[2].Dot(v));
	}

	//代数余子式
	inline float CoFactor(int r1, int c1, int r2, int c2) const
	{
		return Ele[r1][c1] * Ele[r2][c2] - Ele[r1][c2] * Ele[r2][c1];
	}

	inline TATMatrix3 Inverse() const
	{
		TATVector3 co = TATVector3(CoFactor(1, 1, 2, 2), CoFactor(1, 2, 2, 0), CoFactor(1, 0, 2, 1));
		float det = Ele[0].Dot(co);
		assert(det != float(0.0));
		float s = float(1.0) / det;
		return TATMatrix3(co.X * s, CoFactor(0, 2, 2, 1) * s, CoFactor(0, 1, 1, 2) * s,
						  co.Y * s, CoFactor(0, 0, 2, 2) * s, CoFactor(0, 2, 1, 0) * s,
						  co.Z * s, CoFactor(0, 1, 2, 0) * s, CoFactor(0, 0, 1, 1) * s);
	}

	void SetRotation(const TATQuaternion& q)
	{
		float d = q.Length2();
		assert(d != float(0.0));
		float s = float(2.0) / d;

		float xs = q.X * s, ys = q.Y * s, zs = q.Z * s;
		float wx = q.W * xs, wy = q.W * ys, wz = q.W * zs;
		float xx = q.X * xs, xy = q.X * ys, xz = q.X * zs;
		float yy = q.Y * ys, yz = q.Y * zs, zz = q.Z * zs;
		SetValue(
			float(1.0) - (yy + zz), xy - wz, xz + wy,
			xy + wz, float(1.0) - (xx + zz), yz - wx,
			xz - wy, yz + wx, float(1.0) - (xx + yy));
	}

	static const TATMatrix3& GetIdentity()
	{
		static const TATMatrix3
			identityMatrix(
				float(1.0), float(0.0), float(0.0),
				float(0.0), float(1.0), float(0.0),
				float(0.0), float(0.0), float(1.0));

		return identityMatrix;
	}

	void GetRotation(TATQuaternion& q) const
	{
		float trace = Ele[0].X + Ele[1].Y + Ele[2].Z;

		float temp[4];

		if (trace > float(0.0))
		{
			float s = sqrt(trace + float(1.0));
			temp[3] = (s * float(0.5));
			s = float(0.5) / s;

			temp[0] = ((Ele[2].Y - Ele[1].Z) * s);
			temp[1] = ((Ele[0].Z - Ele[2].X) * s);
			temp[2] = ((Ele[1].X - Ele[0].Y) * s);
		}
		else
		{
			int i = Ele[0].X < Ele[1].Y ? (Ele[1].Y < Ele[2].Z ? 2 : 1) : (Ele[0].X < Ele[2].Z ? 2 : 0);
			int j = (i + 1) % 3;
			int k = (i + 2) % 3;

			float s = sqrt(Ele[i][i] - Ele[j][j] - Ele[k][k] + float(1.0));
			temp[i] = s * float(0.5);
			s = float(0.5) / s;

			temp[3] = (Ele[k][j] - Ele[j][k]) * s;
			temp[j] = (Ele[j][i] + Ele[i][j]) * s;
			temp[k] = (Ele[k][i] + Ele[i][k]) * s;
		}
		q.SetValue(temp[0], temp[1], temp[2], temp[3]);
	}

	void GetEulerYPR(float& yaw, float& pitch, float& roll) const
	{
		// first use the normal calculus
		yaw = float(atan2f(Ele[1].X, Ele[0].X));
		pitch = float(asinf(-Ele[2].X));
		roll = float(atan2f(Ele[2].Y, Ele[2].Z));

		// on pitch = +/-HalfPI
		if (fabsf(pitch) == TAT_HALF_PI)
		{
			if (yaw > 0)
				yaw -= TAT_PI;
			else
				yaw += TAT_PI;

			if (roll > 0)
				roll -= TAT_PI;
			else
				roll += TAT_PI;
		}
	};

	inline TATMatrix3 Transpose() const
	{
		return TATMatrix3(Ele[0].X, Ele[1].X, Ele[2].X,
			Ele[0].Y, Ele[1].Y, Ele[2].Y,
			Ele[0].Z, Ele[1].Z, Ele[2].Z);
	}

	inline TATMatrix3 Adjoint() const
	{
		return TATMatrix3(CoFactor(1, 1, 2, 2), CoFactor(0, 2, 2, 1), CoFactor(0, 1, 1, 2),
			CoFactor(1, 2, 2, 0), CoFactor(0, 0, 2, 2), CoFactor(0, 2, 1, 0),
			CoFactor(1, 0, 2, 1), CoFactor(0, 1, 2, 0), CoFactor(0, 0, 1, 1));
	}

	inline TATMatrix3 TransposeTimes(const TATMatrix3& m) const
	{
		return TATMatrix3(
			Ele[0].X * m[0].X + Ele[1].X * m[1].X + Ele[2].X * m[2].X,
			Ele[0].X * m[0].Y + Ele[1].X * m[1].Y + Ele[2].X * m[2].Y,
			Ele[0].X * m[0].Z + Ele[1].X * m[1].Z + Ele[2].X * m[2].Z,
			Ele[0].Y * m[0].X + Ele[1].Y * m[1].X + Ele[2].Y * m[2].X,
			Ele[0].Y * m[0].Y + Ele[1].Y * m[1].Y + Ele[2].Y * m[2].Y,
			Ele[0].Y * m[0].Z + Ele[1].Y * m[1].Z + Ele[2].Y * m[2].Z,
			Ele[0].Z * m[0].X + Ele[1].Z * m[1].X + Ele[2].Z * m[2].X,
			Ele[0].Z * m[0].Y + Ele[1].Z * m[1].Y + Ele[2].Z * m[2].Y,
			Ele[0].Z * m[0].Z + Ele[1].Z * m[1].Z + Ele[2].Z * m[2].Z);
	}

	void GetOpenGLSubMatrix(float* m) const
	{
		m[0] = float(Ele[0].X);
		m[1] = float(Ele[1].X);
		m[2] = float(Ele[2].X);
		m[3] = float(0.0);
		m[4] = float(Ele[0].Y);
		m[5] = float(Ele[1].Y);
		m[6] = float(Ele[2].Y);
		m[7] = float(0.0);
		m[8] = float(Ele[0].Z);
		m[9] = float(Ele[1].Z);
		m[10] = float(Ele[2].Z);
		m[11] = float(0.0);
	}
};

inline TATVector3 operator*(const TATVector3& v, const TATMatrix3& m)
{
	return TATVector3(
		m.Ele[0].X * v.X + m.Ele[1].X * v.Y + m.Ele[2].X * v.Z,
		m.Ele[0].Y * v.X + m.Ele[1].Y * v.Y + m.Ele[2].Y * v.Z,
		m.Ele[0].Z * v.X + m.Ele[1].Z * v.Y + m.Ele[2].Z * v.Z
	);
}