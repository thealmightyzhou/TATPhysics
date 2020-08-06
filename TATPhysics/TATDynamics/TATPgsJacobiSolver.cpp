#include "TATPgsJacobiSolver.h"
#include "TATRigidBodyData.h"
#include "TATDynamicWorld.h"
#include "../TATNarrowPhase/TATSAT.h"
#include "../TATNarrowPhase/TATRigidBodyCollideData.h"

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

void TATPgsJacobiSolver::GetContactPoint(const TATRigidBodyCollideData& contact, TATContactPoint& out)
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

	out.m_PositionWorldOnA = contact.m_CollidePt0;
	out.m_PositionWorldOnB = contact.m_CollidePt1;
}

//Impulse(k+1) = Impulse(k) + J-1 * R;
//R : J * Impulse(k) - J * v;
//J-1 * R : J-1 * J * Impulse(k) - J-1 * J * V;
//J-1 : m_jacDiagABInv £¨JM-1JT£©-1
//sum : Impulse(k+1)
//m_appliedImpulse : Impulse(k)
//deltaImpulse : J-1 * R = -R ?
//deltaVel1Dotn & deltaVel2Dotn : J * V;
//-= deltaVel1Dotn * c.m_jacDiagABInv : - J-1 * J * V;
//c.m_rhs - b3Scalar(c.m_appliedImpulse) * c.m_cfm; : J-1 * J * Impulse(k)
//rhs = M * (b(pos) + b(vel)); M¡÷V
//cfm : some threshold;
void TATPgsJacobiSolver::ResolveSingleConstraintRowGeneric(TATSolverBody& body1, TATSolverBody& body2, const TATSolverConstraint& c)
{
	float deltaImpulse = c.m_Rhs - float(c.m_AppliedImpulse) * c.m_Cfm;

	//J * V
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

	body1.ApplyImpulse(-c.m_ContactNormal * body1.GetInvMass(), c.m_AngularComponentA, deltaImpulse);
	body2.ApplyImpulse(c.m_ContactNormal * body2.GetInvMass(), c.m_AngularComponentB, deltaImpulse);
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

	TATRigidBody& rb0 = TATDynamicWorld::Instance()->m_RigidBodys[bodyA.m_OriginalBodyIndex];
	TATRigidBody& rb1 = TATDynamicWorld::Instance()->m_RigidBodys[bodyB.m_OriginalBodyIndex];

	bodyA.m_MassCenter = rb0.GetMassCenter();
	bodyB.m_MassCenter = rb1.GetMassCenter();

	TATRigidBodyData* rbData0 = &bodies[bodyA.m_OriginalBodyIndex];
	TATRigidBodyData* rbData1 = &bodies[bodyB.m_OriginalBodyIndex];

	rel_pos1 = pos1 - bodyA.m_MassCenter;
	rel_pos2 = pos2 - bodyB.m_MassCenter;

	relaxation = 1.f;

	TATVector3 torqueAxis0 = rel_pos1.Cross(cp.m_NormalWorldOnA);
	solverConstraint.m_AngularComponentA = rbData0 ? TATDynamicWorld::Instance()->m_InertiaDatas[rb0.m_InertiaIndex].m_InvInertiaWorld * torqueAxis0 : TATVector3::Zero();
	TATVector3 torqueAxis1 = rel_pos2.Cross(cp.m_NormalWorldOnA);
	solverConstraint.m_AngularComponentB = rbData1 ? TATDynamicWorld::Instance()->m_InertiaDatas[rb1.m_InertiaIndex].m_InvInertiaWorld * -torqueAxis1 : TATVector3::Zero();

	float scaledDenom;

	TATVector3 vec;
	float denom0 = 0.f;
	float denom1 = 0.f;
	if (rbData0)
	{
		vec = (solverConstraint.m_AngularComponentA).Cross(rel_pos1);
		denom0 = rbData0->m_InvMass + cp.m_NormalWorldOnA.Dot(vec);
	}
	if (rbData1)
	{
		vec = (-solverConstraint.m_AngularComponentB).Cross(rel_pos2);
		denom1 = rbData1->m_InvMass + cp.m_NormalWorldOnA.Dot(vec);
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

	vel1 = rbData0 ? GetVelocityInLocalPoint(rbData0, rel_pos1) : TATVector3::Zero();
	vel2 = rbData1 ? GetVelocityInLocalPoint(rbData1, rel_pos2) : TATVector3::Zero();

	vel = vel1 - vel2;
	relVel = cp.m_NormalWorldOnA.Dot(vel);

	solverConstraint.m_Friction = cp.m_CombinedFriction;

	restitution = RestitutionCurve(relVel, cp.m_CombinedRestitution);//CHECK
	if (restitution <= float(0.))
	{
		restitution = 0.f;
	};


	solverConstraint.m_AppliedImpulse = cp.m_AppliedImpulse * infoGlobal.m_WarmstartingFactor;
	//if (rbData0)
	//	bodyA.ApplyImpulse(solverConstraint.m_ContactNormal * bodyA.GetInvMass(), solverConstraint.m_AngularComponentA, solverConstraint.m_AppliedImpulse);
	//if (rbData1)
	//	bodyB.ApplyImpulse(solverConstraint.m_ContactNormal * bodyB.GetInvMass(), -solverConstraint.m_AngularComponentB, -(float)solverConstraint.m_AppliedImpulse);

	if (rbData0)
		bodyA.ApplyImpulse(solverConstraint.m_ContactNormal * bodyA.GetInvMass(), -solverConstraint.m_AngularComponentA, -solverConstraint.m_AppliedImpulse);
	if (rbData1)
		bodyB.ApplyImpulse(solverConstraint.m_ContactNormal * bodyB.GetInvMass(), solverConstraint.m_AngularComponentB, solverConstraint.m_AppliedImpulse);

	solverConstraint.m_AppliedPushImpulse = 0.f;

	float vel1Dotn = solverConstraint.m_ContactNormal.Dot(rbData0 ? bodyA.m_LinVel : TATVector3::Zero()) + solverConstraint.m_RelPos1CrossNormal.Dot(rbData0 ? bodyA.m_AngVel : TATVector3::Zero());
	float vel2Dotn = -solverConstraint.m_ContactNormal.Dot(rbData1 ? bodyB.m_LinVel : TATVector3::Zero()) + solverConstraint.m_RelPos2CrossNormal.Dot(rbData1 ? bodyB.m_AngVel : TATVector3::Zero());
	float rel_vel = vel1Dotn + vel2Dotn;

	float positionalError = 0.f;
	float velocityError = restitution + rel_vel;  // * damping;

	float erp = infoGlobal.m_Erp2;
	if (!infoGlobal.m_SplitImpulse || (penetration > infoGlobal.m_SplitImpulsePenetrationThreshold))
	{
		erp = infoGlobal.m_Erp;
	}

	if (penetration < 0)
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

void TATPgsJacobiSolver::SolveContact(const TATRigidBodyCollideData& contact, TATRigidBodyData* bodies, TATInertiaData* inertias, TATContactSolverInfo& info)
{
	info.m_SplitImpulse = false;
	TATVector3 vel;
	TATVector3 relPos1;
	TATVector3 resPos2;
	float relVel;
	float relaxation;

	TATRigidBody& rb0 = TATDynamicWorld::Instance()->m_RigidBodys[contact.m_RbIndex0];
	TATRigidBody& rb1 = TATDynamicWorld::Instance()->m_RigidBodys[contact.m_RbIndex1];

	int bodyIndex0 = rb0.m_BodyIndex;
	int bodyIndex1 = rb1.m_BodyIndex;

	int dataIndex0 = rb0.m_DataIndex;
	int dataIndex1 = rb1.m_DataIndex;

	TATSolverBody& bodyA = m_SolverBodyPool[bodyIndex0];
	TATSolverBody& bodyB = m_SolverBodyPool[bodyIndex1];

	TATRigidBodyData& data0 = bodies[dataIndex0];
	TATRigidBodyData& data1 = bodies[dataIndex1];

	bodyA.SetWorldTransform(TATransform(data0.m_Quat, data0.m_Pos));
	bodyA.m_LinVel = data0.m_LinVel;
	bodyA.m_AngVel = data0.m_AngVel;

	bodyB.SetWorldTransform(TATransform(data1.m_Quat, data1.m_Pos));
	bodyB.m_LinVel = data1.m_LinVel;
	bodyB.m_AngVel = data1.m_AngVel;

	TATSolverConstraint constr;
	TATContactPoint cp;
	GetContactPoint(contact, cp);

	SetupContactConstraint(bodies, inertias, constr, bodyIndex0, bodyIndex1,
		cp, info, vel, relVel, relaxation, relPos1, resPos2);

	for (int i = 0; i < m_IterateNum; ++i)
	{
		ResolveSingleConstraintRowGeneric(bodyA, bodyB, constr);
	}

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