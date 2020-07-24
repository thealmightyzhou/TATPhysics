#pragma once
#include "../TATCommon/TATMatrix3.h"

struct TATRigidBodyData
{
	TATVector3 m_Pos;
	TATQuaternion m_Quat;
	TATVector3 m_LinVel;
	TATVector3 m_AngVel;

	float m_InvMass;
	float m_RestituitionCoeff;
	float m_FrictionCoeff;
};

struct TATInertiaData
{
	TATMatrix3 m_InvInertiaWorld;
	TATMatrix3 m_InitInvInertia;
};