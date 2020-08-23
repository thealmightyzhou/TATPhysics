#include "TATPBDWorld.h"
#include "TATPBDConstraint.h"
#include "TATPBDBody.h"
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATNarrowPhase/TATSoftRigidCollisionEntry.h"
#include "../TATBroadPhase/TATBvhCollideCallBack.h"

bool TATPBDWorld::AddBody(const TString& name, TATPBDBody* body)
{
	if (m_PhyBodys.find(name) == m_PhyBodys.end())
	{
		m_PhyBodys.insert(std::make_pair(name, body));
		return true;
	}

	else
		return false;
}

bool TATPBDWorld::AddConstraint(TATPBDConstraint* constr)
{
	if (m_Constraints.find(constr->GetHashValue()) == m_Constraints.end())
	{
		m_Constraints.insert(std::make_pair(constr->GetHashValue(), constr));
		return true;
	}
	return false;
}

void TATPBDWorld::RemoveConstraint(int hash)
{
	m_Constraints.erase(hash);
}

TATPBDConstraint* TATPBDWorld::GetConstraint(int hash)
{
	if (m_Constraints.find(hash) != m_Constraints.end())
		return m_Constraints[hash];
	else
		return 0;
}



void TATPBDWorld::StepSimulation(float dt)
{
	std::map<TString, TATPBDBody*>::iterator ite = m_PhyBodys.begin();
	while (ite != m_PhyBodys.end())
	{
		ite->second->StepSimulation(dt);
		ite++;
	}

	m_SoftRigidCollideDatas.clear();

	std::map<int, TATPBDConstraint*>::iterator it;
	for (int i = 0; i < m_IterateNum; ++i)
	{
		it = m_Constraints.begin();
		while (it != m_Constraints.end())
		{
			it->second->SolveConstraint();
			it++;
		}

		//process collision here?
	}

	GenerateCollision(dt);
	ProcessCollision(dt);

	ite = m_PhyBodys.begin();
	while (ite != m_PhyBodys.end())
	{
		ite->second->Integrate(dt);
		ite++;
	}

	ite = m_PhyBodys.begin();
	while (ite != m_PhyBodys.end())
	{
		ite->second->SolveConstraintEnd();
		ite++;
	}
}

class TATSoftRigidCollisionProcessor:public TATBvhCollideCallBack
{
public:
	TATSoftRigidCollisionProcessor(std::vector<TATSoftRigidCollideData>& collidedatas, float dt) :
		m_CollideDatas(collidedatas), m_DeltaTime(dt)
	{}

	virtual void NodeOverlapped(TATBVNode* node1, TATBVNode* node2)
	{
		TATPBDBody* soft = (TATPBDBody*)node1->m_Data;
		TATRigidBody* rigid = (TATRigidBody*)(node2->m_Data);

		std::vector<TATSoftRigidCollideData> datas;
		if (TATSoftRigidCollisionEntry::ProcessSoftRigidCollision(soft, rigid, m_DeltaTime, datas))
		{
			m_CollideDatas.insert(m_CollideDatas.end(), datas.begin(), datas.end());
		}
	}

	std::vector<TATSoftRigidCollideData>& m_CollideDatas;
	float m_DeltaTime;
};

void TATPBDWorld::GenerateCollision(float dt)
{
	m_PhyBodyBVH.Clear();
	std::map<TString, TATPBDBody*>::iterator ite = m_PhyBodys.begin();
	TATVector3 min, max;
	while (ite != m_PhyBodys.end())
	{
		ite->second->UpdateAabb();
		ite->second->m_ParticleBVH.GetBound(min, max);
		TATBVNode* node = m_PhyBodyBVH.InsertAabbNode(min, max);
		node->m_Data = (void*)(ite->second);
		ite++;
	}
	m_PhyBodyBVH.FinishBuild();

	TATSoftRigidCollisionProcessor cb(m_SoftRigidCollideDatas, dt);
	TATDynamicWorld::Instance()->m_RigidBodyBVTree.CollideWithBVTree(&m_PhyBodyBVH, &cb); //TODO refactory
}

void TATPBDWorld::ProcessCollision(float dt)
{
	for (int i = 0; i < m_SoftRigidCollideDatas.size(); ++i)
	{
		ProjectRSCollision(m_SoftRigidCollideDatas[i], dt);

		//float impulse_sum = 0.0f;
		//SolveCollision(m_SoftRigidCollideDatas[i], dt, impulse_sum, 1.0, 0);
	}
}

void TATPBDWorld::ProjectRSCollision(const TATSoftRigidCollideData& data, float dt)
{
	TATPBDParticle* particle = data.m_Particle;
	TATRigidBody* rigid = data.m_Rigid;

	if (rigid->m_InvMass != 0)
	{
		TATransform predictTr;
		TATransformUtil::IntegrateTransform(rigid->GetWorldTransform(), rigid->GetLinearVelocity(), rigid->GetAngularVelocity(), data.m_HitFraction * dt, predictTr);
		rigid->SetWorldTransform(predictTr);
		rigid->UpdataInverseInertiaWorld();
	}

	const TATMatrix3& iwi = TATDynamicWorld::Instance()->m_InertiaDatas[rigid->m_InertiaIndex].m_InvInertiaWorld;
	TATVector3 r = data.m_RigidPt - rigid->GetMassCenter();
	//TATVector3 r = data.m_SoftPt - rigid->GetMassCenter();

	TATMatrix3 impulseMatrix = TATCollisionUtil::ImpulseMatrix
	(
		dt,
		data.m_Particle->m_InvMass,
		rigid->m_InvMass,
		iwi, 
		r
	);

	TATVector3 va = rigid->GetVelocityAtWCS(data.m_RigidPt) * dt;
	//TATVector3 va = rigid->GetVelocityAtWCS(data.m_SoftPt) * dt;
	TATVector3 vb = particle->m_PredictPos - particle->Position();
	const TATVector3 rel_vel = vb - va;
	const float rel_vel_normal = rel_vel.Dot(data.m_CollideNormal);
	if (rel_vel_normal > 0)
		return;

	const TATVector3 rel_frict_vel = rel_vel - rel_vel_normal * data.m_CollideNormal;
	float frict = particle->m_HostBody->m_FrictionCoeffcient * rigid->m_FrictionCoefficient;
	float fricCoeff = rel_frict_vel.Length() < rel_vel_normal * particle->m_HostBody->m_FrictionCoeffcient ? 0 : 1 - frict;

	float kst = 1.0f;

	const TATVector3 impulse = impulseMatrix *
		(rel_vel - rel_frict_vel * fricCoeff - data.m_Penetration * rigid->m_ContactHardness * data.m_CollideNormal) * kst;

	particle->m_PredictPos = data.m_SoftPt;
	particle->m_PredictPos -= impulse * particle->m_InvMass * dt;
	//particle->m_Velocity -= impulse * particle->m_InvMass;

	rigid->ApplyImpulse(impulse, r);

}

void TATPBDWorld::ComputeMatrixK(const TATVector3& connector, float invMass, const TATVector3& x, TATMatrix3& iwi, TATMatrix3& K)
{
	if (invMass != 0)
	{
		const TATVector3 v = connector - x;
		const float a = v[0];
		const float b = v[1];
		const float c = v[2];

		// J is symmetric
		const float j11 = iwi(0, 0);
		const float j12 = iwi(0, 1);
		const float j13 = iwi(0, 2);
		const float j22 = iwi(1, 1);
		const float j23 = iwi(1, 2);
		const float j33 = iwi(2, 2);

		//K(0, 0) = c * c * j22 - b * c * (j23 + j23) + b * b * j33 + invMass;
		//K(0, 1) = -(c * c * j12) + a * c * j23 + b * c * j13 - a * b * j33;
		//K(0, 2) = b * c * j12 - a * c * j22 - b * b * j13 + a * b * j23;
		//K(1, 0) = K(0, 1);
		//K(1, 1) = c * c * j11 - a * c * (j13 + j13) + a * a * j33 + invMass;
		//K(1, 2) = -(b * c * j11) + a * c * j12 + a * b * j13 - a * a * j23;
		//K(2, 0) = K(0, 2);
		//K(2, 1) = K(1, 2);
		//K(2, 2) = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + invMass;

		K(0, 0) = c * c * j22 - b * c * (j23 + j23) + b * b * j33 + invMass;
		K(1, 0) = -(c * c * j12) + a * c * j23 + b * c * j13 - a * b * j33;
		K(2, 0) = b * c * j12 - a * c * j22 - b * b * j13 + a * b * j23;
		K(0, 1) = K(1, 0);
		K(1, 1) = c * c * j11 - a * c * (j13 + j13) + a * a * j33 + invMass;
		K(2, 1) = -(b * c * j11) + a * c * j12 + a * b * j13 - a * a * j23;
		K(0, 2) = K(2, 0);
		K(1, 2) = K(2, 1);
		K(2, 2) = b * b * j11 - a * b * (j12 + j12) + a * a * j22 + invMass;
	}
	else
		K.SetZero();
}

//not works :< (pick from pbd master)
void TATPBDWorld::SolveCollision(const TATSoftRigidCollideData& data, float dt,float &sumImpulse,float stiff,float frictionStiff)
{
	TATPBDParticle* particle = data.m_Particle;
	TATRigidBody* rigid = data.m_Rigid;

	TATransform predictTr;
	TATransformUtil::IntegrateTransform(rigid->GetWorldTransform(), rigid->GetLinearVelocity(), rigid->GetAngularVelocity(), data.m_HitFraction * dt, predictTr);
	rigid->SetWorldTransform(predictTr);
	rigid->UpdataInverseInertiaWorld();

	TATMatrix3 k;
	ComputeMatrixK(data.m_RigidPt, rigid->m_InvMass, rigid->GetMassCenter(), TATDynamicWorld::Instance()->m_InertiaDatas[rigid->m_InertiaIndex].m_InvInertiaWorld, k);
	if (particle->m_InvMass != 0.0)
	{
		k(0, 0) += particle->m_InvMass;
		k(1, 1) += particle->m_InvMass;
		k(2, 2) += particle->m_InvMass;
	}

	float nknInv = static_cast<float>(1.0) / (data.m_CollideNormal.Dot(k * data.m_CollideNormal));

	const TATVector3 u1 = rigid->GetVelocityAtWCS(data.m_RigidPt);
	const TATVector3 u_rel = particle->m_Velocity - u1;
	const float u_rel_n = data.m_CollideNormal.Dot(u_rel);
	TATVector3 t = u_rel - u_rel_n * data.m_CollideNormal;
	float tl2 = t.Length2();
	if (tl2 > 1.0e-6)
		t *= static_cast<float>(1.0) / sqrt(tl2);

	float maxImpulseTan = static_cast<float>(1.0) / (t.Dot(k * t)) * u_rel.Dot(t);

	float restitutionCoeff = 0.9;

	float targetVel = 0.0;
	if (u_rel_n < 0.0)
		targetVel = -restitutionCoeff * u_rel_n;

	const TATVector3& connector0 = data.m_SoftPt;
	const TATVector3& connector1 = data.m_RigidPt;
	const TATVector3& normal = data.m_CollideNormal;
	const TATVector3& tangent = t;

	// penetration depth 
	const float d = normal.Dot(connector0 - connector1);

	const float delta_u_reln = targetVel - u_rel_n;

	float correctionMagnitude = nknInv * delta_u_reln;

	if (correctionMagnitude < -sumImpulse)
		correctionMagnitude = -sumImpulse;

	// add penalty impulse to counteract penetration
	if (d < 0.0)
		correctionMagnitude -= stiff * nknInv * d;

	TATVector3 p(correctionMagnitude * normal);
	sumImpulse += correctionMagnitude;

	const float pn = p.Dot(normal);
	if (frictionStiff * pn > maxImpulseTan)
		p -= maxImpulseTan * tangent;
	else if (frictionStiff * pn < -maxImpulseTan)
		p += maxImpulseTan * tangent;
	else
		p -= frictionStiff * pn * tangent;

	if (particle->m_InvMass != 0.0)
	{
		particle->m_Velocity += particle->m_InvMass * p;
	}

	rigid->ApplyImpulse(-p, data.m_RigidPt);

}