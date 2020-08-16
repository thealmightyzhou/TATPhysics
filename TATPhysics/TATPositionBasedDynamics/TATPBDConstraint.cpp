#include "TATPBDConstraint.h"
#include "TATPBDBody.h"

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

