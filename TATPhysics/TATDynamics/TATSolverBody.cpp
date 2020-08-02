#include "TATSolverBody.h"

TATSolverBody::TATSolverBody()
{
	Clear();
}

void TATSolverBody::Clear()
{
	m_LinVel.SetZero();
	m_AngVel.SetZero();
	m_WorldTransform.SetIdentity();
	m_InvMass.SetZero();
	m_DeltaAngVel.SetZero();
	m_DeltaLinVel.SetZero();
	m_PushVel.SetZero();
	m_TurnVel.SetZero();
	m_LinFactor.SetValue(1, 1, 1);
	m_AngFactor.SetValue(1, 1, 1);
}