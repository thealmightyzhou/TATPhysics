#include "TATPBDConstraint.h"
#include "TATPBDBody.h"
#include "../TATDynamics/TATRigidBody.h"

TATPBDVertexDistConstraint::TATPBDVertexDistConstraint(TATPBDParticle* v0, TATPBDParticle* v1, float compressStiff, float strechStiff) :
	m_Vertex0(v0), m_Vertex1(v1),
	m_CompressionStiffness(compressStiff), m_StrechStiffness(strechStiff)
{
	m_RestValue = v0->Position().Distance(v1->Position());
	m_HashValue = TATHasher::HashTwo((int)v0, (int)v1);
}

bool TATPBDVertexDistConstraint::SolveConstraint()
{
	float wSum = m_Vertex0->m_InvMass + m_Vertex1->m_InvMass;
	if (wSum == 0.0)
		return false;

	TATVector3 n = m_Vertex1->m_PredictPos - m_Vertex0->m_PredictPos;
	float d = n.Length();
	n.SafeNormalize();

	TATVector3 corr;
	if (d < m_RestValue)
		corr = m_CompressionStiffness * n * (d - m_RestValue) / wSum;
	else
		corr = m_StrechStiffness * n * (d - m_RestValue) / wSum;

	m_Vertex0->m_PredictPos += m_Vertex0->m_InvMass * corr;
	m_Vertex1->m_PredictPos += -m_Vertex1->m_InvMass * corr;
	return true;
}

//----------------------------

TATPBDVertexSpacePtDistConstraint::TATPBDVertexSpacePtDistConstraint(TATPBDParticle* v, const TATVector3& pt, float comp, float strech) :
	m_Vertex(v), m_Position(pt), m_CompressionStiffness(comp), m_StrechStiffness(strech)
{
	m_RestValue = v->Position().Distance(pt);
	m_HashValue = TATHasher::HashThree((int)v * pt.X, (int)v * pt.Y, (int)v * pt.Z);
}

bool TATPBDVertexSpacePtDistConstraint::SolveConstraint()
{
	float wSum = m_Vertex->m_InvMass;
	if (wSum == 0.0)
		return false;

	TATVector3 n = m_Position - m_Vertex->m_PredictPos;
	float d = n.Length();
	n.SafeNormalize();

	TATVector3 corr;
	if (d < m_RestValue)
		corr = m_CompressionStiffness * n * (d - m_RestValue) / wSum;
	else
		corr = m_StrechStiffness * n * (d - m_RestValue) / wSum;

	m_Vertex->m_PredictPos += m_Vertex->m_InvMass * corr;
	return true;
}

//----------------------------

TATPBDVertexRigidPtDistConstraint::TATPBDVertexRigidPtDistConstraint(TATPBDParticle* v, const TATVector3& ptInLocal, TATRigidBody* rigid, float comp, float strech) :
	m_Vertex(v), m_Position(ptInLocal), m_Rigid(rigid), m_CompressionStiffness(comp), m_StrechStiffness(strech)
{
	m_RestValue = v->Position().Distance(rigid->GetWorldTransform() * ptInLocal);
	m_HashValue = TATHasher::HashThree((int)v * (int)rigid, (int)v + (int)rigid, (int)v * (ptInLocal.X + ptInLocal.Y + ptInLocal.Z));
}

bool TATPBDVertexRigidPtDistConstraint::SolveConstraint()
{
	float wSum = m_Vertex->m_InvMass + m_Rigid->m_InvMass;
	if (wSum == 0.0)
		return false;
	const TATVector3 pos = m_Rigid->GetWorldTransform() * m_Position;
	TATVector3 n = pos - m_Vertex->m_PredictPos;
	float d = n.Length();
	n.SafeNormalize();

	TATVector3 corr;
	if (d < m_RestValue)
		corr = m_CompressionStiffness * n * (d - m_RestValue) / wSum;
	else
		corr = m_StrechStiffness * n * (d - m_RestValue) / wSum;

	m_Vertex->m_PredictPos += m_Vertex->m_InvMass * corr;
	//m_Vertex1->m_PredictPos += -m_Vertex1->m_InvMass * corr;
	return true;
}

//----------------------------

TATPBDVolumeConstraint::TATPBDVolumeConstraint(TATPBDParticle* p0, TATPBDParticle* p1, TATPBDParticle* p2, TATPBDParticle* p3, float negStiff, float posStiff) :
	m_Vertex0(p0), m_Vertex1(p1), m_Vertex2(p2), m_Vertex3(p3),
	m_NegtiveStiffness(negStiff), m_PositiveStiffness(posStiff)
{
	m_RestValue = static_cast<float>(1.0 / 6.0)* (p1->Position() - p0->Position()).Cross(p2->Position() - p0->Position()).Dot(p3->Position() - p0->Position());
	m_HashValue = TATHasher::HashTwo((int)p0 + (int)p1 + (int)p2 + (int)p3, (int)p0 * (int)p1 * (int)p2 * (int)p3);
}

bool TATPBDVolumeConstraint::SolveConstraint()
{
	float volume = static_cast<float>(1.0 / 6.0)* (m_Vertex1->m_PredictPos - m_Vertex0->m_PredictPos)
		.Cross(m_Vertex2->m_PredictPos - m_Vertex0->m_PredictPos).Dot(m_Vertex3->m_PredictPos - m_Vertex0->m_PredictPos);

	if (m_PositiveStiffness == 0.0 && volume > 0.0)
		return false;

	if (m_NegtiveStiffness == 0.0 && volume < 0.0)
		return false;


	TATVector3 grad0 = (m_Vertex1->m_PredictPos - m_Vertex2->m_PredictPos).Cross(m_Vertex3->m_PredictPos - m_Vertex2->m_PredictPos);
	TATVector3 grad1 = (m_Vertex2->m_PredictPos - m_Vertex0->m_PredictPos).Cross(m_Vertex3->m_PredictPos - m_Vertex0->m_PredictPos);
	TATVector3 grad2 = (m_Vertex0->m_PredictPos - m_Vertex1->m_PredictPos).Cross(m_Vertex3->m_PredictPos - m_Vertex1->m_PredictPos);
	TATVector3 grad3 = (m_Vertex1->m_PredictPos - m_Vertex0->m_PredictPos).Cross(m_Vertex2->m_PredictPos - m_Vertex0->m_PredictPos);

	float lambda =
		m_Vertex0->m_InvMass * grad0.Length2() +
		m_Vertex1->m_InvMass * grad1.Length2() +
		m_Vertex2->m_InvMass * grad2.Length2() +
		m_Vertex3->m_InvMass * grad3.Length2();

	if (fabs(lambda) < TAT_EPSILON2)
		return false;

	if (volume < 0.0)
		lambda = m_NegtiveStiffness * (volume - m_RestValue) / lambda;
	else
		lambda = m_PositiveStiffness * (volume - m_RestValue) / lambda;

	m_Vertex0->m_PredictPos += -lambda * m_Vertex0->m_InvMass * grad0;
	m_Vertex1->m_PredictPos += -lambda * m_Vertex1->m_InvMass * grad1;
	m_Vertex2->m_PredictPos += -lambda * m_Vertex2->m_InvMass * grad2;
	m_Vertex3->m_PredictPos += -lambda * m_Vertex3->m_InvMass * grad3;

	return true;
}

//----------------------------

TATPBDDihedralConstraint::TATPBDDihedralConstraint(TATPBDParticle* p0, TATPBDParticle* p1, TATPBDParticle* p2, TATPBDParticle* p3, float stiff) :
	m_Particle0(p0), m_Particle1(p1), m_Particle2(p2), m_Particle3(p3), m_Stiffness(stiff)
{
	TATVector3 e = p3->m_PredictPos - p2->m_PredictPos;
	float eLen = e.Length();
	float inv_eLen = static_cast<float>(1.0) / eLen;

	TATVector3 n1 = (p2->m_PredictPos - p0->m_PredictPos).Cross(p3->m_PredictPos - p0->m_PredictPos);
	TATVector3 n2 = (p3->m_PredictPos - p1->m_PredictPos).Cross(p2->m_PredictPos - p1->m_PredictPos);

	n1.SafeNormalize();
	n2.SafeNormalize();

	float dot = n1.Dot(n2);

	if (dot < -1.0) dot = -1.0;
	if (dot > 1.0) dot = 1.0;
	m_RestValue = acos(dot);
}

bool TATPBDDihedralConstraint::SolveConstraint()
{
	TATVector3 e = m_Particle3->m_PredictPos - m_Particle2->m_PredictPos;
	float eLen = e.Length();
	float inv_eLen = static_cast<float>(1.0) / eLen;
	TATVector3 n1 = (m_Particle2->m_PredictPos - m_Particle0->m_PredictPos).Cross(m_Particle3->m_PredictPos - m_Particle0->m_PredictPos);
	//n1.SafeNormalize(); 
	n1 /= n1.Length2();
	TATVector3 n2 = (m_Particle3->m_PredictPos - m_Particle1->m_PredictPos).Cross(m_Particle2->m_PredictPos - m_Particle1->m_PredictPos);
	//n2.SafeNormalize();
	n2 /= n2.Length2();

	TATVector3 d0 = eLen * n1;
	TATVector3 d1 = eLen * n2;
	TATVector3 d2 = (m_Particle0->m_PredictPos - m_Particle3->m_PredictPos).Dot(e) * inv_eLen * n1
		+ (m_Particle1->m_PredictPos - m_Particle3->m_PredictPos).Dot(e) * inv_eLen * n2;
	TATVector3 d3 = (m_Particle2->m_PredictPos - m_Particle0->m_PredictPos).Dot(e) * inv_eLen * n1 
		+ (m_Particle2->m_PredictPos - m_Particle1->m_PredictPos).Dot(e) * inv_eLen * n2;

	n1.SafeNormalize();
	n2.SafeNormalize();

	float dot = n1.Dot(n2);

	if (dot < -1.0) dot = -1.0;
	if (dot > 1.0) dot = 1.0;
	float phi = acos(dot);

	float lambda = m_Particle0->m_InvMass * d0.Length2() +
		m_Particle1->m_InvMass * d1.Length2() +
		m_Particle2->m_InvMass * d2.Length2() +
		m_Particle3->m_InvMass * d3.Length2();

	if (lambda == 0.0)
		return false;

	lambda = (phi - m_RestValue) / lambda * m_Stiffness;

	if (n1.Cross(n2).Dot(e) > 0.0)
		lambda = -lambda;

	m_Particle0->m_PredictPos += -m_Particle0->m_InvMass * lambda * d0;
	m_Particle1->m_PredictPos += -m_Particle1->m_InvMass * lambda * d1;
	m_Particle2->m_PredictPos += -m_Particle2->m_InvMass * lambda * d2;
	m_Particle3->m_PredictPos += -m_Particle3->m_InvMass * lambda * d3;
}