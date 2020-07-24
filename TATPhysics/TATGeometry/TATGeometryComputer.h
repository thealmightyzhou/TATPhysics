#pragma once
#include "../TATCommon/TATVector3.h"

class TATGeometryUtil
{
public:
	static TATVector3 ClosetPtOnPlane(const TATVector3& p, const TATVector3& f, const TATVector3& d)
	{
		TATVector3 fp = p - f;
		if (fp.Length() < TAT_EPSILON2)
			return f;

		float proj = fp.Dot(d);
		TATVector3 a = fp - proj * d;
		return f + a;
	}

	static bool IsPtInTri(const TATVector3& p, const TATVector3& t0, const TATVector3& t1, const TATVector3& t2)
	{
		TATVector3 e0 = t1 - t0;
		TATVector3 e1 = t2 - t1;
		TATVector3 e2 = t0 - t2;

		TATVector3 normal = e0.Cross(e1);

		TATVector3 p0 = e0.Cross(t0 - p);
		TATVector3 p1 = e1.Cross(t1 - p);
		TATVector3 p2 = e2.Cross(t2 - p);

		bool jug0 = p0.Dot(normal) >= 0;
		bool jug1 = p1.Dot(normal) >= 0;
		bool jug2 = p2.Dot(normal) >= 0;

		if (jug0 != jug1 || jug1 != jug2 || jug0 != jug2)
			return false;
		else
			return true;
	}

	static TATVector3 ClosetPtOnTri(const TATVector3& pt, const TATVector3& a, const TATVector3& b, const TATVector3& c,float* weight)
	{
		TATVector3 ab = b - a;
		TATVector3 ac = c - a;

		TATVector3 p = ClosetPtOnPlane(pt, ab.Cross(ac).Normalized(), a);

		TATVector3 ap = p - a;
		float d1 = ab.Dot(ap);
		float d2 = ac.Dot(ap);
		if (d1 <= .0f && d2 <= .0f)
			return a;

		TATVector3 bp = p - b;
		float d3 = ab.Dot(bp);
		float d4 = ac.Dot(bp);
		if (d3 >= .0f && d4 <= d3)
			return b;

		TATVector3 cp = p - c;
		float d5 = ab.Dot(cp);
		float d6 = ac.Dot(cp);
		if (d6 >= .0f && d5 < d6)
			return c;

		float vc = d1 * d4 - d3 * d2;
		if (vc <= .0f && d1 >= .0f && d3 <= .0f)
		{
			float v1 = d1 / (d1 - d3);
			return a + v1 * ab;
		}

		float vb = d5 * d2 - d1 * d6;
		if (vb <= .0f && d2 >= .0f && d6 <= .0f)
		{
			float w1 = d2 / (d2 - d6);
			return a + w1 * ac;
		}

		float va = d3 * d6 - d5 * d4;
		if (va <= .0f && (d4 - d3) >= .0f && (d5 - d6) >= .0f)
		{
			float w1 = (d4 - d3) / ((d4 - d3) + (d5 - d6));
			return b + w1 * (c - b);
		}

		float denom = 1.0f / (va + vb + vc);
		float v = vb * denom;
		float w = vc * denom;

		return a + ab * v + ac * w;
	}

	static TATVector3 ClosetPtOnSegment(const TATVector3& p, const TATVector3& e0, const TATVector3& e1)
	{
		TATVector3 seg = e1 - e0;
		TATVector3 dir = seg.Normalized();
		float len = seg.Length();
		TATVector3 ep = p - e0;

		float proj = ep.Dot(dir);
		if (proj < 0)
			return e0;
		if (proj > len)
			return e1;

		return (proj * dir + e0);
	}

	//segments intersect on the plane which they made
	static void ClosetPtBetweenIntersectSegments(const TATVector3& a, const TATVector3& b, const TATVector3& c, const TATVector3& d, TATVector3& p1, TATVector3& p2)
	{
		TATVector3 d_ = b + d - c;
		TATVector3 bd_ = d_ - b;
		float projbd_ = bd_.Dot((a - b).Normalized());
		float hbd_ = sqrt(bd_.Length2() - projbd_ * projbd_);

		TATVector3 bd = d - b;
		float projbd = bd.Dot((a - b).Normalized());
		float hbd = sqrt(bd.Length2() - projbd * projbd);

		TATVector3 dc = c - d;
		TATVector3 ptOncd = d + (hbd / hbd_) * dc.Length() * dc.Normalized();
		p2 = ptOncd;

		TATVector3 cd = d - c;

		TATVector3 a_ = c + a - b;
		TATVector3 ca_ = a_ - c;
		float projca_ = ca_.Dot(cd.Normalized());
		float hca_ = sqrt(ca_.Length2() - projca_ * projca_);

		TATVector3 ca = a - c;
		float projca = ca.Dot(cd.Normalized());
		float hca = sqrt(ca.Length2() - projca * projca);
		TATVector3 ab = b - a;

		TATVector3 ptOnab = a + (hca / hca_) * ab.Length() * ab.Normalized();
		p1 = ptOnab;
	}
};

class TATVoronoiUtil
{
public:
	static bool PtInFaceVor()
	{
		//TODO
		return false;
	}
};