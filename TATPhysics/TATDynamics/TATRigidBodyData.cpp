#include "TATRigidBodyData.h"

TATRigidBodyData::TATRigidBodyData()
{
	Clear();
}

void TATRigidBodyData::Clear()
{
	m_Pos.SetZero();
	m_Quat.SetIdentity();
	m_LinVel.SetZero();
	m_AngVel.SetZero();

	m_InvMass = 0.0f;
	m_RestituitionCoeff = 0.0f;
	m_FrictionCoeff = 0.0f;
}

TATInertiaData::TATInertiaData()
{
	Clear();
}

void TATInertiaData::Clear()
{
	m_InvInertiaWorld.SetIdentity();
	m_InitInvInertia.SetIdentity();
}