#pragma once
#include "../TATCommon/TATVector3.h"
#include "../TATMath/TATMatrix.h"

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

	//TODO fill weight
	static TATVector3 ClosetPtOnTri(const TATVector3& pt, const TATVector3& a, const TATVector3& b, const TATVector3& c,float* weight)
	{
		TATVector3 ab = b - a;
		TATVector3 ac = c - a;

		TATVector3 p = ClosetPtOnPlane(pt, a, ab.Cross(ac).Normalized());

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

	static float PtTriDist(const TATVector3& pt, const TATVector3& a, const TATVector3& b, const TATVector3& c)
	{
		float weight[3];
		TATVector3 ptOnTri = ClosetPtOnTri(pt, a, b, c, weight);
		return pt.Distance(ptOnTri);

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

	//segpt0 = @weight0 * a + (1 - @weight0) * b;
	//segpt1 = @weight1 * c + (1 - @weight1) * d;
	static float GetSegmentsClosetPt(const TATVector3& a, const TATVector3& b, const TATVector3& c, const TATVector3& d, float& weight0, float& weight1)
	{
		TATMatrix A(3, 2);
		TATVector3 ba = a - b;
		TATVector3 cd = d - c;
		float ab_len2 = ba.Length2();
		float cd_len2 = cd.Length2();

		//parallel
		if (fabs(ba.Normalized().Dot(cd.Normalized())) > 0.99f)
		{
			TATVector3 dir = -ba.Normalized();
			float proj0[2]{ a.Dot(dir),b.Dot(dir) };
			float proj1[2]{ c.Dot(dir),d.Dot(dir) };

			TATVector3 c_d[2]{ c,d };

			int swapcd = 0;
			if (proj1[0] > proj1[1])
			{
				swap(proj1[0], proj1[1]);
				swapcd = 1;
			}

			if (proj0[1] <= proj1[0])
			{
				weight0 = 0;
				weight1 = 1 - swapcd;
				return b.Distance(c_d[0 + swapcd]);
			}
			else if (proj1[1] <= proj0[0])
			{
				weight0 = 1;
				weight1 = 0 + swapcd;
				return a.Distance(c_d[1 - swapcd]);
			}
			else if (proj1[0] <= proj0[0] && proj0[1] <= proj1[1])
			{
				TATVector3 mid = (a + b) / 2;
				TATVector3 pt = ClosetPtOnSegment(mid, c, d);
				TATVector3 f = pt - c;
				weight0 = 0.5f;
				weight1 = 1 - (f.Dot(cd) / cd_len2);
				return pt.Distance(mid);
			}
			else if (proj0[0] <= proj1[0] && proj1[1] <= proj0[1])
			{
				TATVector3 mid = (c + d) / 2;
				TATVector3 pt = ClosetPtOnSegment(mid, a, b);
				TATVector3 f = pt - a;
				weight0 = 1 - f.Dot(-ba) / ab_len2;
				weight1 = 0.5f;
				return pt.Distance(mid);
			}
			else if (proj0[0] < proj1[0])
			{
				TATVector3 pt = ClosetPtOnSegment(c_d[0 + swapcd], a, b);
				TATVector3 f = pt - a;
				float w0 = 1 - f.Dot(-ba) / ab_len2;
				float w1 = 1 - swapcd;

				pt = ClosetPtOnSegment(b, c_d[0 + swapcd], c_d[1 - swapcd]);
				f = pt - c;
				float w2 = 0;
				float w3 = 1 - (f.Dot(cd) / (cd.Dot(cd)));

				weight0 = (w0 + w2) / 2;
				weight1 = (w1 + w3) / 2;

				return (a * weight0 + (1 - weight0) * b).Distance(c * weight1 + (1 - weight1) * d);

			}
			else if (proj1[0] < proj0[0])
			{
				TATVector3 pt = ClosetPtOnSegment(c_d[1 - swapcd], a, b);
				TATVector3 f = pt - a;
				float w0 = 1 - f.Dot(-ba) / ab_len2;
				float w1 = 0 + swapcd;

				pt = ClosetPtOnSegment(a, c_d[1 - swapcd], c_d[0 + swapcd]);
				f = pt - c;
				float w2 = 1;
				float w3 = 1 - (f.Dot(cd) / cd_len2);

				weight0 = (w0 + w2) / 2;
				weight1 = (w1 + w3) / 2;

				return (a * weight0 + (1 - weight0) * b).Distance(c * weight1 + (1 - weight1) * d);
			}
		}
		else
		{
			A(0, 0) = ba[0]; A(1, 0) = ba[1]; A(2, 0) = ba[2];
			A(0, 1) = cd[0]; A(1, 1) = cd[1]; A(2, 1) = cd[2];

			TATMatrix a_transpose = A.Transpose();
			TATVector3 bd = d - b;
			TATMatrix bd_mat(3, 1);
			bd_mat(0, 0) = bd[0]; bd_mat(1, 0) = bd[1]; bd_mat(2, 0) = bd[2];

			TATMatrix right_part = a_transpose * bd_mat;
			TATMatrix left_part = (a_transpose * A).Inverse();
			TATMatrix res = left_part * right_part;

			weight0 = res.Get(0, 0);
			weight1 = res.Get(1, 0);

			_Clamp(weight0, 0.0f, 1.0f);
			_Clamp(weight1, 0.0f, 1.0f);

			TATVector3 p = a * weight0 + b * (1 - weight0);
			TATVector3 q = c * weight1 + d * (1 - weight1);
			return p.Distance(q);
		}
	}
};

class TATVoronoiUtil
{
public:
	static bool PtInFaceVor(const TATVector3& p, const TATVector3& p0, const TATVector3& p1, const TATVector3& p2)
	{
		bool isInTri = TATGeometryUtil::IsPtInTri(p, p0, p1, p2);
		bool isInPostive = (p-p0).Dot((p1 - p0).Cross(p2 - p0)) > 0;
		if (isInTri && isInPostive)
			return true;
		return false;
	}
};