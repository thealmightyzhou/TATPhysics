#pragma once
#include "TATSolverConstraint.h"
#include "TATRigidBodyData.h"
#include "TATRigidBody.h"
#include <vector>

struct TATRigidBodyCollideData;

struct TATContactPoint
{
	TATVector3 m_NormalWorldB2A;
	float m_AppliedImpulse;
	float m_CombinedRestitution;

	///information related to friction
	float m_CombinedFriction;
	TATVector3 m_LateralFrictionDir1;
	TATVector3 m_LateralFrictionDir2;
	float m_AppliedImpulseLateral1;
	float m_AppliedImpulseLateral2;
	float m_CombinedRollingFriction;
	float m_ContactMotion1;
	float m_ContactMotion2;
	float m_ContactCFM1;
	float m_ContactCFM2;

	bool m_LateralFrictionInitialized;

	TAT_REGISTER_ATTRIBUTE_GET(TATVector3, PositionWorldOnA);
	TAT_REGISTER_ATTRIBUTE_GET(TATVector3, PositionWorldOnB);
	TAT_REGISTER_ATTRIBUTE_GET(float, Distance);

};

class TATPgsJacobiSolver
{
public:
	TATPgsJacobiSolver():m_SolverBodyPool(TAT_MAXRIGIDBODY_COUNT), m_SolverContactConstraintPool(TAT_MAXSOLVERCONSTRAINT_COUNT)
	{
		m_IterateNum = 4;
	}

	TATObjectPool<TATSolverBody> m_SolverBodyPool;
	
	TATObjectPool<TATSolverConstraint> m_SolverContactConstraintPool;

	void ResolveSingleConstraintRowGeneric(TATSolverBody* bodyA, TATSolverBody* bodyB, const TATSolverConstraint& contactConstraint);

	void SetupContactConstraint(TATRigidBodyData* bodies, TATInertiaData* inertias, TATSolverConstraint& solverConstraint,
		int solverBodyIdA, int solverBodyIdB,
		TATContactPoint& cp, const TATContactSolverInfo& infoGlobal,
		TATVector3& vel, float& rel_vel, float& relaxation,
		TATVector3& rel_pos1, TATVector3& rel_pos2);

	float RestitutionCurve(float rel_vel, float restitution);

	void GetContactPoint(const TATRigidBodyCollideData& contact, TATContactPoint& out);

	void SolveContact(const TATRigidBodyCollideData& contact, TATRigidBodyData* bodies, TATInertiaData* inertias, TATContactSolverInfo& info);

	void SolveFinish(TATRigidBodyData* bodies, TATInertiaData* inertias, const TATContactSolverInfo& info);

	void InitSolverBody(int bodyIndex, const TATRigidBodyData& data, const TATRigidBody& rb);

	void SetIteNum(int num) 
	{
		m_IterateNum = num;
	}

	int m_IterateNum;
};