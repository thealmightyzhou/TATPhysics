#pragma once
#include "../TATGeometry/TATMeshInfo.h"
#include "../TATResources/TATModelLoader.h"
#include "../TATBasis/TATickable.h"
#include "../TATBroadPhase/TATBvh.h"

class TATActor;
class TATPBDConstraint;
class TATPBDBody;

struct TATPBDParticle
{
public:
	TATPBDParticle(float invMass, TATPhyVertex& vertex, TATPBDBody* body) :
		m_InvMass(invMass), m_PhyVertex(vertex), m_HostBody(body)
	{
		m_Velocity.SetZero();
		m_ExternalForce.SetZero();
		m_PredictPos = m_PhyVertex.m_Position;
	}

	float m_InvMass;
	TATVector3 m_Velocity;
	TATVector3 m_ExternalForce;
	TATVector3 m_InstantForce;
	TATVector3 m_ConstantForce;
	TATVector3 m_PredictPos;
	TATPhyVertex& m_PhyVertex;
	TATPBDBody* m_HostBody;
	int m_Index;

	TATVector3& Position()
	{
		return m_PhyVertex.m_Position;
	}

	void AddInstantForce(const TATVector3& f)
	{
		m_InstantForce += f;
	}

	void AddConstantForce(const TATVector3& f)
	{
		m_ConstantForce += f;
	}
};

class TATPBDBody:public TATickable
{
public:

	TATPBDBody(const TString& name, const TATModelBuffer& buffer, float invMassPerNode);

	virtual ~TATPBDBody() {}

	void Build(const TATModelBuffer& buffer,float invMassPerNode)
	{
		m_PhyBody.m_Vertices.resize(buffer.vertexBuffer.size());
		m_PhyBody.m_Faces.resize(buffer.faceBuffer.size());
		m_PhyBody.m_Tetras.resize(buffer.tetraBuffer.size());

		for (int i = 0; i < m_PhyBody.m_Vertices.size(); ++i)
		{
			m_PhyBody.m_Vertices[i].m_Position = buffer.vertexBuffer[i].m_Position;
			if (buffer.vertexBuffer[i].m_IsRendVert)
				m_RenderParticles.push_back(i);
		}

		for (int i = 0; i < m_PhyBody.m_Faces.size(); ++i)
		{
			m_PhyBody.m_Faces[i].m_VertexIndices[0] = buffer.faceBuffer[i].v1;
			m_PhyBody.m_Faces[i].m_VertexIndices[1] = buffer.faceBuffer[i].v2;
			m_PhyBody.m_Faces[i].m_VertexIndices[2] = buffer.faceBuffer[i].v3;
		}

		for (int i = 0; i < m_PhyBody.m_Tetras.size(); ++i)
		{
			m_PhyBody.m_Tetras[i].m_VertexIndices[0] = buffer.tetraBuffer[i].v1;
			m_PhyBody.m_Tetras[i].m_VertexIndices[1] = buffer.tetraBuffer[i].v2;
			m_PhyBody.m_Tetras[i].m_VertexIndices[2] = buffer.tetraBuffer[i].v3;
			m_PhyBody.m_Tetras[i].m_VertexIndices[3] = buffer.tetraBuffer[i].v4;
		}

		m_Particles.reserve(m_PhyBody.m_Vertices.size());
		for (int i = 0; i < m_PhyBody.m_Vertices.size(); i++)
		{
			m_Particles.push_back(TATPBDParticle(invMassPerNode, m_PhyBody.m_Vertices[i], this));
			m_Particles[i].m_Index = i;
		}
	}

	void Initialize();

	virtual bool Update(TATActor* actor, float dt);

	virtual void AddPositionConstraint(float compress,float strech);

	virtual void AddVolumeConstraint(float neg,float pos);

	virtual void AddConstraint(TATPBDConstraint* constr);

	virtual void StepSimulation(float dt);

	virtual void Integrate(float dt);

	virtual void DampVelocity(TATPBDParticle& particle, float dt);

	void UpdateAabb();

	void SetGravity(const TATVector3& g)
	{
		for (int i = 0; i < m_Particles.size(); i++)
		{
			m_Particles[i].AddConstantForce(g);
		}
	}

	void SetDamping(float lin, float ang)
	{
		m_LinearDamp = lin;
		m_AngularDamp = ang;
	}

	std::vector<TATPBDParticle> m_Particles;

	std::vector<int> m_RenderParticles;

	std::vector<int> m_Constraints;

	TATPhyMeshData m_PhyBody;

	float m_LinearDamp;

	float m_AngularDamp;

	TATBvh m_ParticleBVH;

	float m_FrictionCoeffcient;
};