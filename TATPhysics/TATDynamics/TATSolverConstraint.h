#pragma once
#include "TATSolverBody.h"

class TATSolverConstraint
{
public:
	TATVector3		m_RelPos1CrossNormal;
	TATVector3		m_RelPos2CrossNormal;
	TATVector3		m_ContactNormal;

	TATVector3		m_AngularComponentA;
	TATVector3		m_AngularComponentB;

	mutable float	m_AppliedPushImpulse;
	mutable float	m_AppliedImpulse;
	float			m_JacDiagABInv;
	float			m_Rhs;
	float			m_Cfm;
	float			m_LowerLimit;
	float			m_UpperLimit;
	float			m_RhsPenetration;
	float			m_Friction;

	int				m_OverrideNumSolverIterations;
	int				m_SolverBodyId1;
	int				m_SolverBodyId2;

};

struct TATContactSolverInfo
{
	float m_Tau;
	float m_Damping;  //global non-contact constraint damping, can be locally overridden by constraints during 'getInfo2'.
	float m_Friction;
	float m_TimeStep;
	float m_Restitution;
	int m_NumIterations;
	float m_MaxErrorReduction;
	float m_Sor;
	float m_Erp;        //used as Baumgarte factor
	float m_Erp2;       //used in Split Impulse
	float m_GlobalCfm;  //constraint force mixing
	int m_SplitImpulse;
	float m_SplitImpulsePenetrationThreshold;
	float m_SplitImpulseTurnErp;
	float m_LinearSlop;
	float m_WarmstartingFactor;

	//int m_SolverMode;
	int m_RestingContactRestitutionThreshold;
	int m_MinimumSolverBatchSize;
	float m_MaxGyroscopicForce;
	float m_SingleAxisRollingFrictionThreshold;

	TATContactSolverInfo()
	{
		m_Tau = float(0.6);
		m_Damping = float(1.0);
		m_Friction = float(0.3);
		m_TimeStep = float(1.f / 60.f);
		m_Restitution = float(0.);
		m_MaxErrorReduction = float(20.);
		m_NumIterations = 10;
		m_Erp = float(0.2);
		m_Erp2 = float(0.8);
		m_GlobalCfm = float(0.);
		m_Sor = float(1.);
		m_SplitImpulse = true;
		m_SplitImpulsePenetrationThreshold = -.04f;
		m_SplitImpulseTurnErp = 0.1f;
		m_LinearSlop = float(0.0);
		m_WarmstartingFactor = float(0.85);
		//m_solverMode =  B3_SOLVER_USE_WARMSTARTING |  B3_SOLVER_SIMD | B3_SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION|B3_SOLVER_USE_2_FRICTION_DIRECTIONS|B3_SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;// | B3_SOLVER_RANDMIZE_ORDER;
		//m_SolverMode = B3_SOLVER_USE_WARMSTARTING | B3_SOLVER_SIMD;  // | B3_SOLVER_RANDMIZE_ORDER;
		m_RestingContactRestitutionThreshold = 2;                    //unused as of 2.81
		m_MinimumSolverBatchSize = 128;                              //try to combine islands until the amount of constraints reaches this limit
		m_MaxGyroscopicForce = 100.f;                                ///only used to clamp forces for bodies that have their B3_ENABLE_GYROPSCOPIC_FORCE flag set (using b3RigidBody::setFlag)
		m_SingleAxisRollingFrictionThreshold = 1e30f;                ///if the velocity is above this threshold, it will use a single constraint row (axis), otherwise 3 rows.
	}
};