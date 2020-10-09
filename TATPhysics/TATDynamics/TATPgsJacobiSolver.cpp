#include "TATPgsJacobiSolver.h"
#include "TATDynamicWorld.h"
#include "../TATNarrowPhase/TATSAT.h"
#include "../TATNarrowPhase/TATRigidBodyCollideData.h"

//default use pgs & warm start

void TATPgsJacobiSolver::GetContactPoint(const TATRigidBodyCollideData& contact, TATContactPoint& out)
{
	out.m_AppliedImpulse = 0.f;
	out.m_AppliedImpulseLateral1 = 0.f;
	out.m_AppliedImpulseLateral2 = 0.f;
	out.m_CombinedFriction = contact.GetFrictionCoeff();
	out.m_CombinedRestitution = 0.0f; //TODO
	out.m_CombinedRollingFriction = 0.f;
	out.m_ContactCFM1 = 0.f;
	out.m_ContactCFM2 = 0.f;
	out.m_ContactMotion1 = 0.f;
	out.m_ContactMotion2 = 0.f;
	out.m_Distance = contact.m_Penetration;  //??0.01f

	out.m_NormalWorldB2A = contact.m_CollideNormal.Normalized();

	out.m_NormalWorldB2A.PlaneSpace(out.m_LateralFrictionDir1, out.m_LateralFrictionDir2);
	out.m_LateralFrictionInitialized = true;

	out.m_PositionWorldOnA = contact.m_CollidePt0;
	out.m_PositionWorldOnB = contact.m_CollidePt1;
}

//Impulse(k+1) = Impulse(k) + J-1 * R;
//R : J * Impulse(k) - J * v;
//J-1 * R : J-1 * J * Impulse(k) - J-1 * J * V;
//J-1 : m_jacDiagABInv （JM-1JT）-1
//sum : Impulse(k+1)
//m_appliedImpulse : Impulse(k)
//deltaImpulse : J-1 * R = -R ?
//deltaVel1Dotn & deltaVel2Dotn : J * V;
//-= deltaVel1Dotn * c->m_jacDiagABInv : - J-1 * J * V;
//c->m_rhs - b3Scalar(c->m_appliedImpulse) * c->m_cfm; : J-1 * J * Impulse(k)
//rhs = M * (b(pos) + b(vel)); M△V
//cfm : constraint force mixing
void TATPgsJacobiSolver::ResolveSingleConstraintRowGeneric(TATSolverBody* body1, TATSolverBody* body2, TATSolverConstraint* c)
{
	//整个过程可能是对Rhs进行拟合 使总冲量达到rhs
	//rhs 是 误差量 (velocityImpulse + positionImpulse) * (JM-1JT)-1 总冲量
	float deltaImpulse = c->m_Rhs - float(c->m_AppliedImpulse) * c->m_Cfm;

	//J * V
	const float deltaVel1Dotn = c->m_ContactNormal.Dot(body1->GetDeltaLinVel()) + c->m_RelPos1CrossNormal.Dot(body1->GetDeltaAngVel());
	const float deltaVel2Dotn = -c->m_ContactNormal.Dot(body2->GetDeltaLinVel()) + c->m_RelPos2CrossNormal.Dot(body2->GetDeltaAngVel());

	//	const float delta_rel_vel	=	deltaVel1Dotn-deltaVel2Dotn;
	//deltaImpulse可能是个残差向量
	deltaImpulse -= deltaVel1Dotn * c->m_JacDiagABInv; //△mv
	deltaImpulse -= deltaVel2Dotn * c->m_JacDiagABInv; //△mv

	const float sum = float(c->m_AppliedImpulse) + deltaImpulse;
	if (sum < c->m_LowerLimit)
	{
		deltaImpulse = c->m_LowerLimit - c->m_AppliedImpulse;
		c->m_AppliedImpulse = c->m_LowerLimit;
	}
	else if (sum > c->m_UpperLimit)
	{
		deltaImpulse = c->m_UpperLimit - c->m_AppliedImpulse;
		c->m_AppliedImpulse = c->m_UpperLimit;
	}
	else
	{
		c->m_AppliedImpulse = sum;
	}

	body1->ApplyImpulse(c->m_ContactNormal * body1->GetInvMass(), c->m_AngularComponentA, deltaImpulse);
	body2->ApplyImpulse(-c->m_ContactNormal * body2->GetInvMass(), c->m_AngularComponentB, deltaImpulse);
}

void TATPgsJacobiSolver::SetupContactConstraint(TATSolverConstraint* solverConstraint,
	int solverBodyIdA, int solverBodyIdB,
	TATContactPoint& cp, const TATContactSolverInfo& infoGlobal,
	TATVector3& vel, float& relVel, float& relaxation,
	TATVector3& rel_pos1, TATVector3& rel_pos2)
{
	const TATVector3& pos1 = cp.GetPositionWorldOnA();
	const TATVector3& pos2 = cp.GetPositionWorldOnB();

	TATSolverBody& bodyA = m_SolverBodyPool[solverBodyIdA];
	TATSolverBody& bodyB = m_SolverBodyPool[solverBodyIdB];

	TATRigidBody* rb0 = &TATDynamicWorld::Instance()->m_RigidBodys[bodyA.m_OriginalBodyIndex];
	TATRigidBody* rb1 = &TATDynamicWorld::Instance()->m_RigidBodys[bodyB.m_OriginalBodyIndex];

	//update the inertia inv in world
	rb0->UpdataInverseInertiaWorld();
	rb1->UpdataInverseInertiaWorld();

	rel_pos1 = pos1 - bodyA.m_MassCenter;//
	rel_pos2 = pos2 - bodyB.m_MassCenter;

	relaxation = 1.f;

	TATVector3 torqueAxis0 = rel_pos1.Cross(cp.m_NormalWorldB2A);
	solverConstraint->m_AngularComponentA = rb0 ? rb0->m_InvInertiaWorld * torqueAxis0 : TATVector3::Zero();
	TATVector3 torqueAxis1 = rel_pos2.Cross(cp.m_NormalWorldB2A);
	solverConstraint->m_AngularComponentB = rb1 ? rb1->m_InvInertiaWorld * -torqueAxis1 : TATVector3::Zero();

	float scaledDenom;

	TATVector3 vec;
	float denom0 = 0.f;
	float denom1 = 0.f;
	if (rb0)
	{
		vec = (solverConstraint->m_AngularComponentA).Cross(rel_pos1);
		denom0 = rb0->m_InvMass + cp.m_NormalWorldB2A.Dot(vec);
	}
	if (rb1)
	{
		vec = (-solverConstraint->m_AngularComponentB).Cross(rel_pos2);
		denom1 = rb1->m_InvMass + cp.m_NormalWorldB2A.Dot(vec);
	}

	float denom;

	scaledDenom = denom = relaxation / (denom0 + denom1); //pgs

	solverConstraint->m_JacDiagABInv = denom;

	solverConstraint->m_ContactNormal = cp.m_NormalWorldB2A;
	solverConstraint->m_RelPos1CrossNormal = torqueAxis0;
	solverConstraint->m_RelPos2CrossNormal = -torqueAxis1;

	float restitution = 0.f;
	float penetration = cp.GetDistance() - infoGlobal.m_LinearSlop;

	TATVector3 vel1, vel2;

	vel1 = rb0 ? rb0->GetVelocityAt(rel_pos1) : TATVector3::Zero();
	vel2 = rb1 ? rb1->GetVelocityAt(rel_pos2) : TATVector3::Zero();

	vel = vel1 - vel2;
	relVel = cp.m_NormalWorldB2A.Dot(vel);

	solverConstraint->m_Friction = cp.m_CombinedFriction;

	restitution = RestitutionCurve(relVel, cp.m_CombinedRestitution);//CHECK
	if (restitution <= float(0.))
	{
		restitution = 0.f;
	};

	solverConstraint->m_AppliedImpulse = cp.m_AppliedImpulse * infoGlobal.m_WarmstartingFactor;
	if (rb0)
		bodyA.ApplyImpulse(solverConstraint->m_ContactNormal * bodyA.GetInvMass(), solverConstraint->m_AngularComponentA, solverConstraint->m_AppliedImpulse);
	if (rb1)
		bodyB.ApplyImpulse(solverConstraint->m_ContactNormal * bodyB.GetInvMass(), -solverConstraint->m_AngularComponentB, -(float)solverConstraint->m_AppliedImpulse);

	solverConstraint->m_AppliedPushImpulse = 0.f;

	float vel1Dotn = solverConstraint->m_ContactNormal.Dot(rb0 ? bodyA.m_LinVel : TATVector3::Zero()) + solverConstraint->m_RelPos1CrossNormal.Dot(rb0 ? bodyA.m_AngVel : TATVector3::Zero());
	float vel2Dotn = -solverConstraint->m_ContactNormal.Dot(rb1 ? bodyB.m_LinVel : TATVector3::Zero()) + solverConstraint->m_RelPos2CrossNormal.Dot(rb1 ? bodyB.m_AngVel : TATVector3::Zero());
	float rel_vel = vel1Dotn + vel2Dotn;

	float positionalError = 0.f;
	float velocityError = restitution - rel_vel;  // * damping; //!!

	float erp = infoGlobal.m_Erp2;
	if (!infoGlobal.m_SplitImpulse || (penetration > infoGlobal.m_SplitImpulsePenetrationThreshold))
	{
		erp = infoGlobal.m_Erp;
	}

	if (penetration < 0)
	{
		positionalError = 0;

		//if no contact penetration will <0 and reduce the velocityError
		velocityError += penetration / infoGlobal.m_TimeStep; //!!
	}
	else
	{
		positionalError = penetration * erp / infoGlobal.m_TimeStep; //kcor * error
	}

	float penetrationImpulse = positionalError * scaledDenom;  //solverConstraint.m_jacDiagABInv;
	float velocityImpulse = velocityError * scaledDenom;       //solverConstraint.m_jacDiagABInv;

	if (!infoGlobal.m_SplitImpulse || (penetration > infoGlobal.m_SplitImpulsePenetrationThreshold))
	{
		//combine position and velocity into rhs
		solverConstraint->m_Rhs = penetrationImpulse + velocityImpulse;
		solverConstraint->m_RhsPenetration = 0.f;
	}
	else
	{
		//split position and velocity into rhs and m_rhsPenetration
		solverConstraint->m_Rhs = velocityImpulse;
		solverConstraint->m_RhsPenetration = penetrationImpulse;
	}

	solverConstraint->m_Cfm = 0.f;
	solverConstraint->m_LowerLimit = 0;
	solverConstraint->m_UpperLimit = TAT_MAX2;
}

float TATPgsJacobiSolver::RestitutionCurve(float rel_vel, float restitution)
{
	float rest = restitution * -rel_vel;
	return rest;
}

void TATPgsJacobiSolver::InitSolverBody(int bodyIndex, const TATRigidBody& rb)
{
	TATSolverBody& body = m_SolverBodyPool[bodyIndex];
	body.m_AngVel = rb.m_AngVel;
	body.m_InvMass = rb.m_InvMass;
	body.m_LinVel = rb.m_LinVel;
	body.m_WorldTransform.SetOrigin(rb.m_Pos);
	body.m_WorldTransform.SetRotation(rb.m_Quat);
	body.m_MassCenter = rb.GetMassCenter();
	body.m_OriginalBodyIndex = rb.m_IndexInPool;
	body.m_DeltaAngVel.SetZero();
	body.m_DeltaLinVel.SetZero();
	body.m_PushVel.SetZero();
	body.m_TurnVel.SetZero();
	body.m_AngFactor.One();
	body.m_LinFactor.One();
}

void TATPgsJacobiSolver::SolveContact(const TATRigidBodyCollideData& contact, TATContactSolverInfo& info)
{
	info.m_SplitImpulse = false;
	TATVector3 vel;
	TATVector3 relPos1;
	TATVector3 relPos2;
	float relVel;
	float relaxation;

	TATRigidBody& rb0 = TATDynamicWorld::Instance()->m_RigidBodys[contact.m_RbIndex0];
	TATRigidBody& rb1 = TATDynamicWorld::Instance()->m_RigidBodys[contact.m_RbIndex1];

	TATSolverBody* bodyA = m_SolverBodyPool.FetchUnused();
	TATSolverBody* bodyB = m_SolverBodyPool.FetchUnused();

	rb0.m_BodyIndex = bodyA->m_IndexInPool;
	rb1.m_BodyIndex = bodyB->m_IndexInPool;

	InitSolverBody(rb0.m_BodyIndex, rb0);
	InitSolverBody(rb1.m_BodyIndex, rb1);

	TATSolverConstraint* constr = m_SolverContactConstraintPool.FetchUnused();
	TATSolverConstraint* fricConstr0 = m_SolverFrictionConstraintPool.FetchUnused();
	TATSolverConstraint* fricConstr1 = m_SolverFrictionConstraintPool.FetchUnused();

	TATContactPoint cp;
	GetContactPoint(contact, cp);

	SetupContactConstraint(constr, rb0.m_BodyIndex, rb1.m_BodyIndex,
		cp, info, vel, relVel, relaxation, relPos1, relPos2);

	DecomposeContact(vel, relVel, cp.m_NormalWorldB2A, relPos1, relPos2, bodyA, bodyB, fricConstr0, fricConstr1, relaxation, info, cp);

	float totalImpulse;

	for (int i = 0; i < m_IterateNum; ++i)
	{
		ResolveSingleConstraintRowGeneric(bodyA, bodyB, constr);

		totalImpulse = constr->m_AppliedImpulse;
		fricConstr0->m_LowerLimit = -(fricConstr0->m_Friction * totalImpulse);
		fricConstr0->m_UpperLimit = fricConstr0->m_Friction * totalImpulse;
		ResolveSingleConstraintRowGeneric(bodyA, bodyB, fricConstr0);

		fricConstr1->m_LowerLimit = -(fricConstr1->m_Friction * totalImpulse);
		fricConstr1->m_UpperLimit = fricConstr1->m_Friction * totalImpulse;
		ResolveSingleConstraintRowGeneric(bodyA, bodyB, fricConstr1);
	}

	SolveFinish(info);

	m_SolverContactConstraintPool.ReturnUsed(constr);
	m_SolverFrictionConstraintPool.ReturnUsed(fricConstr0);
	m_SolverFrictionConstraintPool.ReturnUsed(fricConstr1);

}

void TATPgsJacobiSolver::PrepareSolve(std::vector<TATRigidBodyCollideData>& contacts)
{
	if (!m_GlobalInfo)
		return;

	m_GlobalInfo->m_SplitImpulse = false;

	TATVector3 vel;
	TATVector3 relPos1;
	TATVector3 relPos2;
	float relVel;
	float relaxation;

	TATSolverBody* bodyA;
	TATSolverBody* bodyB;
	for (int i = 0; i < contacts.size(); ++i)
	{

		TATRigidBody& rb0 = TATDynamicWorld::Instance()->m_RigidBodys[contacts[i].m_RbIndex0];
		TATRigidBody& rb1 = TATDynamicWorld::Instance()->m_RigidBodys[contacts[i].m_RbIndex1];

		bodyA = m_SolverBodyPool.FetchUnused();
		bodyB = m_SolverBodyPool.FetchUnused();

		rb0.m_BodyIndex = bodyA->m_IndexInPool;
		rb1.m_BodyIndex = bodyB->m_IndexInPool;

		bodyA->m_OriginalBodyIndex = contacts[i].m_RbIndex0;
		bodyB->m_OriginalBodyIndex = contacts[i].m_RbIndex1;

		InitSolverBody(rb0.m_BodyIndex, rb0);
		InitSolverBody(rb1.m_BodyIndex, rb1);

		TATSolverConstraint* constr = m_SolverContactConstraintPool.FetchUnused();
		TATSolverConstraint* fricConstr0 = m_SolverFrictionConstraintPool.FetchUnused();
		TATSolverConstraint* fricConstr1 = m_SolverFrictionConstraintPool.FetchUnused();

		TATContactPoint cp;
		GetContactPoint(contacts[i], cp);

		SetupContactConstraint(constr, rb0.m_BodyIndex, rb1.m_BodyIndex,
			cp, *m_GlobalInfo, vel, relVel, relaxation, relPos1, relPos2);

		DecomposeContact(vel, relVel, cp.m_NormalWorldB2A, relPos1, relPos2, bodyA, bodyB, fricConstr0, fricConstr1, relaxation, *m_GlobalInfo, cp);

		SolverContext context;

		context.m_RigidA = &TATDynamicWorld::Instance()->m_RigidBodys[contacts[i].m_RbIndex0];
		context.m_RigidB = &TATDynamicWorld::Instance()->m_RigidBodys[contacts[i].m_RbIndex1];
		context.m_BodyA = bodyA;
		context.m_BodyB = bodyB;
		context.m_Constr = constr;
		context.m_FrictConstr0 = fricConstr0;
		context.m_FrictConstr1 = fricConstr1;
		context.m_AppliedImpulse = 0;
		m_SolverContexts.push_back(context);
	}
}

void TATPgsJacobiSolver::SolveConstraint()
{
	for (int i = 0; i < m_SolverContexts.size(); ++i)
	{
		SolverContext& context = m_SolverContexts[i];
		ResolveSingleConstraintRowGeneric(context.m_BodyA, context.m_BodyB, context.m_Constr);

		context.m_AppliedImpulse = context.m_Constr->m_AppliedImpulse;
		context.m_FrictConstr0->m_LowerLimit = -(context.m_FrictConstr0->m_Friction * context.m_AppliedImpulse);
		context.m_FrictConstr0->m_UpperLimit = context.m_FrictConstr0->m_Friction * context.m_AppliedImpulse;
		ResolveSingleConstraintRowGeneric(context.m_BodyA, context.m_BodyB, context.m_FrictConstr0);

		context.m_FrictConstr1->m_LowerLimit = -(context.m_FrictConstr1->m_Friction * context.m_AppliedImpulse);
		context.m_FrictConstr1->m_UpperLimit = context.m_FrictConstr1->m_Friction * context.m_AppliedImpulse;
		ResolveSingleConstraintRowGeneric(context.m_BodyA, context.m_BodyB, context.m_FrictConstr1);
	}

}

void TATPgsJacobiSolver::Integrate()
{
	for (int i = 0; i < m_SolverContexts.size(); ++i)
	{
		SolverContext& context = m_SolverContexts[i];
		TATSolverBody* bodyA = context.m_BodyA;
		TATSolverBody* bodyB = context.m_BodyB;

		if (m_GlobalInfo->m_SplitImpulse)
		{
			bodyA->WriteBackVelocityAndTransform(m_GlobalInfo->m_TimeStep, m_GlobalInfo->m_SplitImpulseTurnErp);
			bodyB->WriteBackVelocityAndTransform(m_GlobalInfo->m_TimeStep, m_GlobalInfo->m_SplitImpulseTurnErp);
		}
		else
		{
			bodyA->WriteBackVelocity();
			bodyB->WriteBackVelocity();
		}

		TATRigidBody* rbA = context.m_RigidA;
		TATRigidBody* rbB = context.m_RigidB;
		rbA->m_LinVel = bodyA->m_LinVel;
		rbA->m_AngVel = bodyA->m_AngVel;
		rbB->m_LinVel = bodyB->m_LinVel;
		rbB->m_AngVel = bodyB->m_AngVel;
		if (m_GlobalInfo->m_SplitImpulse)
		{
			rbA->m_Pos = bodyA->GetWorldTransform().GetOrigin();
			rbA->m_Quat = bodyA->GetWorldTransform().GetRotation();
			rbB->m_Pos = bodyB->GetWorldTransform().GetOrigin();
			rbB->m_Quat = bodyB->GetWorldTransform().GetRotation();
		}
	}
}

void TATPgsJacobiSolver::SimulationEnd()
{
	m_SolverBodyPool.Clear();
	m_SolverContactConstraintPool.Clear();
	for (int i = 0; i < m_SolverContexts.size(); ++i)
	{
		m_SolverBodyPool.ReturnUsed(m_SolverContexts[i].m_BodyA);
		m_SolverBodyPool.ReturnUsed(m_SolverContexts[i].m_BodyB);
		m_SolverContactConstraintPool.ReturnUsed(m_SolverContexts[i].m_Constr);
		m_SolverFrictionConstraintPool.ReturnUsed(m_SolverContexts[i].m_FrictConstr0);
		m_SolverFrictionConstraintPool.ReturnUsed(m_SolverContexts[i].m_FrictConstr1);
	}
	m_SolverContexts.clear();
}

void TATPgsJacobiSolver::SolveFinish(const TATContactSolverInfo& info)
{
	const std::vector<TATSolverBody*>& bodys = m_SolverBodyPool.FetchAllUsed();

	for (int i = 0; i < (int)bodys.size(); i++)
	{
		TATRigidBody& rb = TATDynamicWorld::Instance()->m_RigidBodys[bodys[i]->m_OriginalBodyIndex];

		TATSolverBody* body = bodys[i];
		if (info.m_SplitImpulse)
			body->WriteBackVelocityAndTransform(info.m_TimeStep, info.m_SplitImpulseTurnErp);
		else
			body->WriteBackVelocity();

		rb.m_LinVel = body->m_LinVel;
		rb.m_AngVel = body->m_AngVel;

		if (info.m_SplitImpulse)
		{
			rb.m_Pos = body->GetWorldTransform().GetOrigin();
			rb.m_Quat = body->GetWorldTransform().GetRotation();
		}
	}

	m_SolverBodyPool.Clear();
}

//default use two friction direction
void TATPgsJacobiSolver::DecomposeContact(
	const TATVector3& vel, const TATVector3& rel_vel, const TATVector3& normal,
	const TATVector3& rel_pos0, const TATVector3& rel_pos1,
	TATSolverBody* bdA, TATSolverBody* bdB,
	TATSolverConstraint* constr0, TATSolverConstraint* constr1,
	float relaxation, const TATContactSolverInfo& info,
	TATContactPoint& cp)
{
	TATVector3 frictionDir0 = vel - normal * rel_vel;
	TATVector3 frictionDir1;

	float frictionLen2 = frictionDir0.Length2();

	if (frictionLen2 > TAT_EPSILON)
	{
		frictionDir0 *= 1.0f / sqrt(frictionLen2);
		frictionDir1 = frictionDir0.Cross(normal);
		frictionDir1.SafeNormalize();

		cp.m_LateralFrictionDir1 = frictionDir0;
		cp.m_LateralFrictionDir2 = frictionDir1;

		SetupFrictionConstraint(bdA, bdB, constr0, cp, frictionDir0, rel_pos0, rel_pos1, relaxation);

		SetupFrictionConstraint(bdA, bdB, constr1, cp, frictionDir1, rel_pos0, rel_pos1, relaxation);
	}

	else
	{
		normal.PlaneSpace(frictionDir0, frictionDir1);

		SetupFrictionConstraint(bdA, bdB, constr0, cp, frictionDir0, rel_pos0, rel_pos1, relaxation);

		SetupFrictionConstraint(bdA, bdB, constr1, cp, frictionDir1, rel_pos0, rel_pos1, relaxation);
	}

	TATRigidBody& rb0 = TATDynamicWorld::Instance()->m_RigidBodys[bdA->m_OriginalBodyIndex];
	TATRigidBody& rb1 = TATDynamicWorld::Instance()->m_RigidBodys[bdB->m_OriginalBodyIndex];

	{
		//default use warm start
		constr0->m_AppliedImpulse = cp.m_AppliedImpulseLateral1 * info.m_WarmstartingFactor;
		if (rb0.m_InvMass)
			bdA->ApplyImpulse(constr0->m_ContactNormal * rb0.m_InvMass, constr0->m_AngularComponentA, constr0->m_AppliedImpulse);
		if (rb1.m_InvMass)
			bdB->ApplyImpulse(constr0->m_ContactNormal * rb1.m_InvMass, -constr0->m_AngularComponentB, -(float)constr0->m_AppliedImpulse);
	}

	{
		constr1->m_AppliedImpulse = cp.m_AppliedImpulseLateral2 * info.m_WarmstartingFactor;
		if (rb0.m_InvMass)
			bdA->ApplyImpulse(constr1->m_ContactNormal * rb0.m_InvMass, constr1->m_AngularComponentA, constr1->m_AppliedImpulse);
		if (rb1.m_InvMass)
			bdB->ApplyImpulse(constr1->m_ContactNormal * rb1.m_InvMass, -constr1->m_AngularComponentB, -(float)constr1->m_AppliedImpulse);
	}
}

void TATPgsJacobiSolver::SetupFrictionConstraint(TATSolverBody* bd0, TATSolverBody* bd1,
	TATSolverConstraint* constr, TATContactPoint& cp,
	const TATVector3& normal,
	const TATVector3& rel_pos0, const TATVector3& rel_pos1, float relaxation, float desiredVel)
{
	constr->m_ContactNormal = normal;
	constr->m_SolverBodyId1 = bd0->m_IndexInPool;
	constr->m_SolverBodyId2 = bd1->m_IndexInPool;
	constr->m_AppliedImpulse = 0;
	constr->m_AppliedPushImpulse = 0;
	constr->m_Friction = cp.m_CombinedFriction;

	TATRigidBody& rb0 = TATDynamicWorld::Instance()->m_RigidBodys[bd0->m_OriginalBodyIndex];
	TATRigidBody& rb1 = TATDynamicWorld::Instance()->m_RigidBodys[bd1->m_OriginalBodyIndex];

	TATVector3 ftorqueAxis1 = rel_pos0.Cross(constr->m_ContactNormal);
	constr->m_RelPos1CrossNormal = ftorqueAxis1;
	constr->m_AngularComponentA = rb0.m_InvMass ? rb0.m_InvInertiaWorld * ftorqueAxis1 : TATVector3::Zero();

	TATVector3 ftorqueAxis2 = rel_pos1.Cross(-constr->m_ContactNormal);
	constr->m_RelPos2CrossNormal = ftorqueAxis2;
	constr->m_AngularComponentB = rb1.m_InvMass ? rb1.m_InvInertiaWorld * ftorqueAxis2 : TATVector3::Zero();

	float scaledDenom;

	{
		TATVector3 vec;
		float denom0 = 0.f;
		float denom1 = 0.f;
		if (rb0.m_InvMass)
		{
			vec = (constr->m_AngularComponentA).Cross(rel_pos0);
			denom0 = rb0.m_InvMass + normal.Dot(vec);
		}
		if (rb1.m_InvMass)
		{
			vec = (-constr->m_AngularComponentB).Cross(rel_pos1);
			denom1 = rb1.m_InvMass + normal.Dot(vec);
		}

		float denom;
		//default use pgs
		scaledDenom = denom = relaxation / (denom0 + denom1);

		constr->m_JacDiagABInv = denom;
	}

	{
		float rel_vel;
		float vel1Dotn = constr->m_ContactNormal.Dot(rb0.m_InvMass ? bd0->m_LinVel : TATVector3::Zero()) + constr->m_RelPos1CrossNormal.Dot(rb0.m_InvMass ? bd0->m_AngVel : TATVector3::Zero());
		float vel2Dotn = -constr->m_ContactNormal.Dot(rb1.m_InvMass ? bd1->m_LinVel : TATVector3::Zero()) + constr->m_RelPos2CrossNormal.Dot(rb1.m_InvMass ? bd1->m_AngVel : TATVector3::Zero());

		rel_vel = vel1Dotn + vel2Dotn;

		//float positionalError = 0.f;

		float velocityError = desiredVel - rel_vel;
		float velocityImpulse = velocityError * float(scaledDenom);  //solverConstraint.m_jacDiagABInv);
		constr->m_Rhs = velocityImpulse;
		//constr.m_cfm = cfmSlip;
		constr->m_LowerLimit = 0;
		constr->m_UpperLimit = TAT_MAX2;
	}
}