#include "TATransformUtil.h"
#include "../TATDynamics/TATRigidBodyData.h"

void TATransformUtil::IntegrateTransform(TATRigidBodyData* body, float timeStep, float angularDamping, const TATVector3& gravityAcc)
{
	if ((body->m_InvMass != 0.f))
	{
		//angular velocity
		{
			TATVector3 axis;

			body->m_AngVel *= angularDamping;

			TATVector3 angvel = body->m_AngVel;
			float fAngle = angvel.Length();
			//limit the angular motion
			if (fAngle * timeStep > TAT_ANGULAR_MOTION_THRESHOLD)
			{
				fAngle = TAT_ANGULAR_MOTION_THRESHOLD / timeStep;
			}
			if (fAngle < 0.001f)
			{
				// use Taylor's expansions of sync function
				axis = angvel * (0.5f * timeStep - (timeStep * timeStep * timeStep) * 0.020833333333f * fAngle * fAngle);
			}
			else
			{
				// sync(fAngle) = sin(c*fAngle)/t
				axis = angvel * (sin(0.5f * fAngle * timeStep) / fAngle);
			}
			TATQuaternion dorn(axis.X, axis.Y, axis.Z, cos(fAngle * timeStep * 0.5f));

			TATQuaternion orn0 = body->m_Quat;

			TATQuaternion predictedOrn = dorn * orn0;
			predictedOrn.Normalize();
			body->m_Quat = predictedOrn;
		}

		//apply gravity
		body->m_LinVel += gravityAcc * timeStep;

		//linear velocity
		body->m_Pos += body->m_LinVel * timeStep;
	}
}