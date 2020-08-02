#pragma once
#include "../TATCommon/TATMatrix3.h"
#include "../TATCommon/TATObjectPool.h"
#include "../TATCommon/TATCore.h"

struct TATRigidBodyData
{
	TAT_POOL_OBJECT(TATRigidBodyData)
	TATVector3 m_Pos;
	TATQuaternion m_Quat;
	TATVector3 m_LinVel;
	TATVector3 m_AngVel;
	TATVector3 m_Gravity;

	float m_InvMass;
	float m_RestituitionCoeff;
	float m_FrictionCoeff;
};

struct TATInertiaData
{
	TAT_POOL_OBJECT(TATInertiaData)
	TATMatrix3 m_InvInertiaWorld;
	TATMatrix3 m_InitInvInertia;
};