#include "TATSolverConstraint.h"

TATSolverConstraint::TATSolverConstraint()
{
	Clear();
}

void TATSolverConstraint::Clear()
{
	m_RelPos1CrossNormal.SetZero();
	m_RelPos2CrossNormal.SetZero();
	m_ContactNormal.SetZero();
	m_AngularComponentA.SetZero();
	m_AngularComponentB.SetZero();

	m_AppliedPushImpulse = 0;
	m_AppliedImpulse = 0;
	m_JacDiagABInv = 0;
	m_Rhs = 0;
	m_Cfm = 0;
	m_LowerLimit = 0;
	m_UpperLimit = TAT_MAX;
	m_RhsPenetration = 0;
	m_Friction = 0;

	m_OverrideNumSolverIterations = 4;
	m_SolverBodyId1 = -1;
	m_SolverBodyId2 = -1;
}