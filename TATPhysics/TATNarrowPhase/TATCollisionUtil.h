#pragma once
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATCore.h"
#include "../TATGeometry/TATGeometryComputer.h"

#define COLLIDE_EPS 0.2f

class TATCollisionUtil
{
public:
	static bool PtCollideFaceContinous
	(
		TATVector3 pt, const TATVector3& vp,
		TATVector3 face[3], TATVector3 vf[3],
		float& t, float iterateNum, float margin
		)
	{
		int iteNum = 0;
		TATVector3 normal = ((face[1] - face[0]).Cross(face[2] - face[0])).SafeNormalize();
		TATVector3 l0 = face[1] - face[0];
		TATVector3 l1 = face[2] - face[0];
		float dist = (pt - face[0]).Dot(normal) - margin;
		//temp only allowed positive collision
		if (dist < 0)
			return false;
		float vel[3];
		vel[0] = vf[0].Dot(normal);
		vel[1] = vf[1].Dot(normal);
		vel[2] = vf[2].Dot(normal);
		float rel_vel;
		if (dist > 0)
			rel_vel = vel[_MaxOfArray<float>(vel, 3)] - vp.Dot(normal);
		else
			rel_vel = vel[_MinOfArray<float>(vel, 3)] - vp.Dot(normal);

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
			if (dis < (margin + COLLIDE_EPS))
			{
				t = dt;
				return true;
			}

			normal = ((face[1] - face[0]).Cross(face[2] - face[0])).SafeNormalize();
			vel[0] = vf[0].Dot(normal);
			vel[1] = vf[1].Dot(normal);
			vel[2] = vf[2].Dot(normal);
			dist = (pt - vf[0]).Dot(normal) - margin;
			if (dist < 0)
				return false;

			if (dist > 0)
				rel_vel = vel[_MaxOfArray<float>(vel, 3)] - vp.Dot(normal);
			else
				rel_vel = vel[_MinOfArray<float>(vel, 3)] - vp.Dot(normal);

			dt += dist / rel_vel;
			if (1.0 < dt || dt < 0.0)
				return false;

			iteNum++;
		}

		return false;
	}
};