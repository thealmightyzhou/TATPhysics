#pragma once
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATCore.h"
#include "../TATGeometry/TATGeometryComputer.h"

#define COLLIDE_EPS 0.02f

class TATCollisionUtil
{
public:
	static bool PtCollideFaceContinous
	(
		TATVector3 pt, const TATVector3& vp,
		TATVector3 face[3], TATVector3 vf[3],
		float& t, int iterateNum, float margin
	)
	{
		TATVector3 originPos = pt;

		int iteNum = 0;
		TATVector3 normal = ((face[1] - face[0]).Cross(face[2] - face[0])).SafeNormalize();
		float dist = (pt - face[0]).Dot(normal) - margin;

		if (dist < 0)
			return false;

		float vel[3];
		vel[0] = vf[0].Dot(normal);
		vel[1] = vf[1].Dot(normal);
		vel[2] = vf[2].Dot(normal);

		float rel_vel = vel[_MaxOfArray<float>(vel, 3)] - vp.Dot(normal);

		float time = dist / rel_vel;
		if (1.0 < time || time < 0.0)
			return false;

		float dt = time;
		while (iteNum < iterateNum)
		{
			pt += vp * dt;
			face[0] += vf[0] * dt;
			face[1] += vf[1] * dt;
			face[2] += vf[2] * dt;

			float dis = TATGeometryUtil::PtTriDist(pt, face[0], face[1], face[2]);
			if (dis < (margin + COLLIDE_EPS) && TATVoronoiUtil::PtInFaceVor(pt, face[0], face[1], face[2]))
			{
				t = time;
				return true;
			}

			normal = ((face[1] - face[0]).Cross(face[2] - face[0])).SafeNormalize();
			vel[0] = vf[0].Dot(normal);
			vel[1] = vf[1].Dot(normal);
			vel[2] = vf[2].Dot(normal);

			dist = (pt - face[0]).Dot(normal) - margin;

			if (dist < -COLLIDE_EPS)
				return false;

			rel_vel = vel[_MaxOfArray<float>(vel, 3)] - vp.Dot(normal);

			dt = dist / rel_vel;
			time += dt;

			if (1.0 + TAT_EPSILON < time || time < -TAT_EPSILON)
				return false;

			iteNum++;
		}

		return false;
	}
};