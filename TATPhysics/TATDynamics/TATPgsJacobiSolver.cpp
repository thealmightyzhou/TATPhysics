#include "TATPgsJacobiSolver.h"
#include "TATRigidBodyData.h"
#include "TATDynamicWorld.h"
#include "../TATNarrowPhase/TATSAT.h"

//default use pgs & warm start
static TATransform GetWorldTransform(TATRigidBodyData* rb)
{
	TATransform newTrans;
	newTrans.SetOrigin(rb->m_Pos);
	newTrans.SetRotation(rb->m_Quat);
	return newTrans;
}

static const TATMatrix3& GetInvInertiaTensorWorld(TATInertiaData* inertia)
{
	return inertia->m_InvInertiaWorld;
}

static const TATVector3& GetLinearVelocity(TATRigidBodyData* rb)
{
	return rb->m_LinVel;
}

static const TATVector3& GetAngularVelocity(TATRigidBodyData* rb)
{
	return rb->m_AngVel;
}

static TATVector3 GetVelocityInLocalPoint(TATRigidBodyData* rb, const TATVector3& rel_pos)
{
	//we also calculate lin/ang velocity for kinematic objects
	return GetLinearVelocity(rb) + GetAngularVelocity(rb).Cross(rel_pos);
}

void TATPgsJacobiSolver::GetContactPoint(const TATSATCollideData& contact, TATContactPoint& out)
{
	out.m_AppliedImpulse = 0.f;
	out.m_AppliedImpulseLateral1 = 0.f;
	out.m_AppliedImpulseLateral2 = 0.f;
	out.m_CombinedFriction = 0.0f; //TODO
	out.m_CombinedRestitution = 0.0f; //TODO
	out.m_CombinedRollingFriction = 0.f;
	out.m_ContactCFM1 = 0.f;
	out.m_ContactCFM2 = 0.f;
	out.m_ContactMotion1 = 0.f;
	out.m_ContactMotion2 = 0.f;
	out.m_Distance = contact.m_Penetration;  //??0.01f
	TATVector3 normalOnA = contact.m_CollideNormal;
	normalOnA.SafeNormalize();  //is this needed?

	TATVector3 l1, l2;
	//b3PlaneSpace1(normalOnB, l1, l2);

	out.m_NormalWorldOnA = normalOnA;

	out.m_LateralFrictionDir1 = TATVector3(0, 0, 0);// l1;
	out.m_LateralFrictionDir2 = TATVector3(0, 0, 0);// l2;
	out.m_LateralFrictionInitialized = true;

	out.m_PositionWorldOnA = contact.m_CollidePtA;
	out.m_PositionWorldOnB = contact.m_CollidePtB;
}

void TATPgsJacobiSolver::ResolveSingleConstraintRowGeneric(TATSolverBody& body1, TATSolverBody& body2, const TATSolverConstraint& c)
{
	float deltaImpulse = c.m_Rhs - float(c.m_AppliedImpulse) * c.m_Cfm;
	const float deltaVel1Dotn = c.m_ContactNormal.Dot(body1.GetDeltaLinVel()) + c.m_RelPos1CrossNormal.Dot(body1.GetDeltaAngVel());
	const float deltaVel2Dotn = -c.m_ContactNormal.Dot(body2.GetDeltaLinVel()) + c.m_RelPos2CrossNormal.Dot(body2.GetDeltaAngVel());

	//	const float delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
	deltaImpulse -= deltaVel1Dotn * c.m_JacDiagABInv;
	deltaImpulse -= deltaVel2Dotn * c.m_JacDiagABInv;

	const float sum = float(c.m_AppliedImpulse) + deltaImpulse;
	if (sum < c.m_LowerLimit)
	{
		deltaImpulse = c.m_LowerLimit - c.m_AppliedImpulse;
		c.m_AppliedImpulse = c.m_LowerLimit;
	}
	else if (sum > c.m_UpperLimit)
	{
		deltaImpulse = c.m_UpperLimit - c.m_AppliedImpulse;
		c.m_AppliedImpulse = c.m_UpperLimit;
	}
	else
	{
		c.m_AppliedImpulse = sum;
	}

	body1.ApplyImpulse(c.m_ContactNormal * body1.GetInvMass(), c.m_AngularComponentA, deltaImpulse);
	body2.ApplyImpulse(-c.m_ContactNormal * body2.GetInvMass(), c.m_AngularComponentB, deltaImpulse);
}

void TATPgsJacobiSolver::SetupContactConstraint(TATRigidBodyData* bodies, TATInertiaData* inertias, TATSolverConstraint& solverConstraint,
	int solverBodyIdA, int solverBodyIdB,
	TATContactPoint& cp, const TATContactSolverInfo& infoGlobal,
	TATVector3& vel, float& relVel, float& relaxation,
	TATVector3& rel_pos1, TATVector3& rel_pos2)
{
	const TATVector3& pos1 = cp.GetPositionWorldOnA();
	const TATVector3& pos2 = cp.GetPositionWorldOnB();

	TATSolverBody& bodyA = m_SolverBodyPool[solverBodyIdA];
	TATSolverBody& bodyB = m_SolverBodyPool[solverBodyIdB];

	TATRigidBodyData* rb0 = &bodies[bodyA.m_OriginalBodyIndex];
	TATRigidBodyData* rb1 = &bodies[bodyB.m_OriginalBodyIndex];

	rel_pos1 = pos1 - bodyA.GetWorldTransform().GetOrigin();
	rel_pos2 = pos2 - bodyB.GetWorldTransform().GetOrigin();

	relaxation = 1.f;

	TATVector3 torqueAxis0 = rel_pos1.Cross(cp.m_NormalWorldOnA);
	solverConstraint.m_AngularComponentA = rb0 ? GetInvInertiaTensorWorld(&inertias[bodyA.m_OriginalBodyIndex]) * torqueAxis0 : TATVector3::Zero();
	TATVector3 torqueAxis1 = rel_pos2.Cross(cp.m_NormalWorldOnA);
	solverConstraint.m_AngularComponentB = rb1 ? GetInvInertiaTensorWorld(&inertias[bodyB.m_OriginalBodyIndex]) * -torqueAxis1 : TATVector3::Zero();

	float scaledDenom;

	TATVector3 vec;
	float denom0 = 0.f;
	float denom1 = 0.f;
	if (rb0)
	{
		vec = (solverConstraint.m_AngularComponentA).Cross(rel_pos1);
		denom0 = rb0->m_InvMass + cp.m_NormalWorldOnA.Dot(vec);
	}
	if (rb1)
	{
		vec = (-solverConstraint.m_AngularComponentB).Cross(rel_pos2);
		denom1 = rb1->m_InvMass + cp.m_NormalWorldOnA.Dot(vec);
	}

	float denom;

	scaledDenom = denom = relaxation / (denom0 + denom1); //pgs

	solverConstraint.m_JacDiagABInv = denom;

	solverConstraint.m_ContactNormal = cp.m_NormalWorldOnA;
	solverConstraint.m_RelPos1CrossNormal = torqueAxis0;
	solverConstraint.m_RelPos2CrossNormal = -torqueAxis1;

	float restitution = 0.f;
	float penetration = cp.GetDistance() + infoGlobal.m_LinearSlop;

	TATVector3 vel1, vel2;

	vel1 = rb0 ? GetVelocityInLocalPoint(rb0, rel_pos1) : TATVector3::Zero();
	vel2 = rb1 ? GetVelocityInLocalPoint(rb1, rel_pos2) : TATVector3::Zero();

	vel = vel1 - vel2;
	relVel = cp.m_NormalWorldOnA.Dot(vel);

	solverConstraint.m_Friction = cp.m_CombinedFriction;

	restitution = RestitutionCurve(relVel, cp.m_CombinedRestitution);
	if (restitution <= float(0.))
	{
		restitution = 0.f;
	};


	solverConstraint.m_AppliedImpulse = cp.m_AppliedImpulse * infoGlobal.m_WarmstartingFactor;
	if (rb0)
		bodyA.ApplyImpulse(solverConstraint.m_ContactNormal * bodyA.GetInvMass(), solverConstraint.m_AngularComponentA, solverConstraint.m_AppliedImpulse);
	if (rb1)
		bodyB.ApplyImpulse(solverConstraint.m_ContactNormal * bodyB.GetInvMass(), -solverConstraint.m_AngularComponentB, -(float)solverConstraint.m_AppliedImpulse);

	solverConstraint.m_AppliedPushImpulse = 0.f;

	float vel1Dotn = solverConstraint.m_ContactNormal.Dot(rb0 ? bodyA.m_LinVel : TATVector3::Zero()) + solverConstraint.m_RelPos1CrossNormal.Dot(rb0 ? bodyA.m_AngVel : TATVector3::Zero());
	float vel2Dotn = -solverConstraint.m_ContactNormal.Dot(rb1 ? bodyB.m_LinVel : TATVector3::Zero()) + solverConstraint.m_RelPos2CrossNormal.Dot(rb1 ? bodyB.m_AngVel : TATVector3::Zero());
	float rel_vel = vel1Dotn + vel2Dotn;

	float positionalError = 0.f;
	float velocityError = restitution - rel_vel;  // * damping;

	float erp = infoGlobal.m_Erp2;
	if (!infoGlobal.m_SplitImpulse || (penetration > infoGlobal.m_SplitImpulsePenetrationThreshold))
	{
		erp = infoGlobal.m_Erp;
	}

	if (penetration > 0)
	{
		positionalError = 0;

		velocityError -= penetration / infoGlobal.m_TimeStep;
	}
	else
	{
		positionalError = -penetration * erp / infoGlobal.m_TimeStep;
	}

	float penetrationImpulse = positionalError * scaledDenom;  //solverConstraint.m_jacDiagABInv;
	float velocityImpulse = velocityError * scaledDenom;       //solverConstraint.m_jacDiagABInv;

	if (!infoGlobal.m_SplitImpulse || (penetration > infoGlobal.m_SplitImpulsePenetrationThreshold))
	{
		//combine position and velocity into rhs
		solverConstraint.m_Rhs = penetrationImpulse + velocityImpulse;
		solverConstraint.m_RhsPenetration = 0.f;
	}
	else
	{
		//split position and velocity into rhs and m_rhsPenetration
		solverConstraint.m_Rhs = velocityImpulse;
		solverConstraint.m_RhsPenetration = penetrationImpulse;
	}
	solverConstraint.m_Cfm = 0.f;
	solverConstraint.m_LowerLimit = 0;
	solverConstraint.m_UpperLimit = TAT_MAX2;
}

float TATPgsJacobiSolver::RestitutionCurve(float rel_vel, float restitution)
{
	float rest = restitution * -rel_vel;
	return rest;
}

void TATPgsJacobiSolver::SolveContact(const TATSATCollideData& contact, TATRigidBodyData* bodies, TATInertiaData* inertias, TATContactSolverInfo& info)
{
	info.m_SplitImpulse = false;
	TATVector3 vel;
	TATVector3 relPos1;
	TATVector3 resPos2;
	float relVel;
	float relaxation;

	int index0 = contact.m_RbA->m_BodyIndex;
	int index1 = contact.m_RbB->m_BodyIndex;

	TATSolverConstraint constr;
	TATContactPoint cp;
	GetContactPoint(contact, cp);

	SetupContactConstraint(bodies, inertias, constr, index0, index1,
		cp, info, vel, relVel, relaxation, relPos1, resPos2);

	ResolveSingleConstraintRowGeneric(m_SolverBodyPool[index0], m_SolverBodyPool[index1], constr);

	SolveFinish(bodies, inertias, info);
}

void TATPgsJacobiSolver::SolveFinish(TATRigidBodyData* bodies, TATInertiaData* inertias, const TATContactSolverInfo& info)
{
	const std::vector<TATSolverBody*>& bodys = m_SolverBodyPool.FetchAllUsed();

	for (int i = 0; i < (int)bodys.size(); i++)
	{
		TATRigidBody& rb = TATDynamicWorld::Instance()->m_RigidBodys[bodys[i]->m_OriginalBodyIndex];

		TATRigidBodyData& data = bodies[rb.m_DataIndex];
		TATSolverBody* body = bodys[i];
		if (info.m_SplitImpulse)
			body->WriteBackVelocityAndTransform(info.m_TimeStep, info.m_SplitImpulseTurnErp);
		else
			body->WriteBackVelocity();

		data.m_LinVel = body->m_LinVel;
		data.m_AngVel = body->m_AngVel;

		if (info.m_SplitImpulse)
		{
			data.m_Pos = body->GetWorldTransform().GetOrigin();
			data.m_Quat = body->GetWorldTransform().GetRotation();
		}
	}
}