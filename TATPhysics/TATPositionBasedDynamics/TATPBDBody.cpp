#include "TATPBDBody.h"
#include "TATPBDConstraint.h"
#include "TATPBDWorld.h"
#include "../TATStage/TATActor.h"
#include "../TATResources/TATMesh.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATApplication/TATApplication.h"
#include "../TATApplication/TAThread.h"
#include "../TATGeometry/TATMeshInfo.h"

TATPBDBody::TATPBDBody(const TString& name, const TATModelBuffer& buffer, float invMassPerNode) :TATickable(name)
{
	Build(buffer, invMassPerNode);

	m_FrictionCoeffcient = 0.9;

	m_UseCCD = false;
}

void TATPBDBody::Initialize()
{
	TATPBDWorld::Instance()->AddBody(m_GlobalName, this);

	TATPhyMeshDataComputer::CompleteMeshData(m_PhyBody.m_Vertices, m_PhyBody.m_Faces, m_PhyBody.m_Edges);
}

void TATPBDBody::AddConstraint(TATPBDConstraint* constr)
{
	if (!TATPBDWorld::Instance()->AddConstraint(constr))
		delete constr;
	else 
		m_Constraints.push_back(constr->GetHashValue());
}

void TATPBDBody::AddPositionConstraint(float compress, float strech)
{
	for (int i = 0; i < m_PhyBody.m_Faces.size(); ++i)
	{
		const TATPhyFace& face = m_PhyBody.m_Faces[i];
		TATPBDConstraint* constr0 = new TATPBDVertexDistConstraint
		(
			&m_Particles[face.m_VertexIndices[0]],
			&m_Particles[face.m_VertexIndices[1]],
			compress, strech
		);
		TATPBDConstraint* constr1 = new TATPBDVertexDistConstraint
		(
			&m_Particles[face.m_VertexIndices[0]],
			&m_Particles[face.m_VertexIndices[2]],
			compress, strech
		);
		TATPBDConstraint* constr2 = new TATPBDVertexDistConstraint
		(
			&m_Particles[face.m_VertexIndices[1]],
			&m_Particles[face.m_VertexIndices[2]],
			compress, strech
		);

		AddConstraint(constr0);
		AddConstraint(constr1);
		AddConstraint(constr2);
	}

	for (int i = 0; i < m_PhyBody.m_Tetras.size(); ++i)
	{
		const TATPhyTetra& tet = m_PhyBody.m_Tetras[i];
		TATPBDConstraint* constr0 = new TATPBDVertexDistConstraint
		(
			&m_Particles[tet.m_VertexIndices[0]],
			&m_Particles[tet.m_VertexIndices[1]],
			compress, strech
		);
		AddConstraint(constr0);

		TATPBDConstraint* constr1 = new TATPBDVertexDistConstraint
		(
			&m_Particles[tet.m_VertexIndices[0]],
			&m_Particles[tet.m_VertexIndices[2]],
			compress, strech
		);
		AddConstraint(constr1);

		TATPBDConstraint* constr2 = new TATPBDVertexDistConstraint
		(
			&m_Particles[tet.m_VertexIndices[0]],
			&m_Particles[tet.m_VertexIndices[3]],
			compress, strech
		);
		AddConstraint(constr2);

		TATPBDConstraint* constr3 = new TATPBDVertexDistConstraint
		(
			&m_Particles[tet.m_VertexIndices[1]],
			&m_Particles[tet.m_VertexIndices[2]],
			compress, strech
		);
		AddConstraint(constr3);

		TATPBDConstraint* constr4 = new TATPBDVertexDistConstraint
		(
			&m_Particles[tet.m_VertexIndices[1]],
			&m_Particles[tet.m_VertexIndices[3]],
			compress, strech
		);
		AddConstraint(constr4);

		TATPBDConstraint* constr5 = new TATPBDVertexDistConstraint
		(
			&m_Particles[tet.m_VertexIndices[2]],
			&m_Particles[tet.m_VertexIndices[3]],
			compress, strech
		);
		AddConstraint(constr5);
	}
}

void TATPBDBody::AddVolumeConstraint(float neg, float pos)
{
	for (int i = 0; i < m_PhyBody.m_Tetras.size(); ++i)
	{
		const TATPhyTetra& tet = m_PhyBody.m_Tetras[i];
		TATPBDVolumeConstraint* constr = new TATPBDVolumeConstraint
		(
			&m_Particles[tet.m_VertexIndices[0]],
			&m_Particles[tet.m_VertexIndices[1]],
			&m_Particles[tet.m_VertexIndices[2]],
			&m_Particles[tet.m_VertexIndices[3]],
			neg, pos
		);

		AddConstraint(constr);
	}
}

void TATPBDBody::AddDihedralConstraint(float stiff)
{
	for (int i = 0; i < m_PhyBody.m_Edges.size(); ++i)
	{
		const TATPhyEdge& e = m_PhyBody.m_Edges[i];

		if (e.m_FaceIndices[0] == INT_MAX || e.m_FaceIndices[1] == INT_MAX)
			continue;

		int face1[3]{ m_PhyBody.m_Faces[e.m_FaceIndices[0]].m_VertexIndices[0] ,
					  m_PhyBody.m_Faces[e.m_FaceIndices[0]].m_VertexIndices[1] ,
					  m_PhyBody.m_Faces[e.m_FaceIndices[0]].m_VertexIndices[2] };

		int face2[3]{ m_PhyBody.m_Faces[e.m_FaceIndices[1]].m_VertexIndices[0] ,
					  m_PhyBody.m_Faces[e.m_FaceIndices[1]].m_VertexIndices[1] ,
					  m_PhyBody.m_Faces[e.m_FaceIndices[1]].m_VertexIndices[2] };

		int points[4];
		int index = 0;
		std::set<int> index_set;
		//make sure only share two vertex
		for (int c = 0; c < 3; ++c)
		{
			if (index_set.find(face1[c]) == index_set.end())
				index_set.insert(face1[c]);
			else
				points[index++] = face1[c];

			if (index_set.find(face2[c]) == index_set.end())
				index_set.insert(face2[c]);
			else
				points[index++] = face2[c];
		}

		for (int c = 0; c < 3; ++c)
		{
			bool repeat = false;
			for (int b = 0; b < index; ++b)
			{
				if (face1[c] == points[b])
					repeat = true;
			}
			if (!repeat)
				points[index++] = face1[c];
			repeat = false;
			for (int b = 0; b < index; ++b)
			{
				if (face2[c] == points[b])
					repeat = true;
			}
			if (!repeat)
				points[index++] = face2[c];
		}

		TATPBDDihedralConstraint* constr = new TATPBDDihedralConstraint(GetParticleAt(points[3]), GetParticleAt(points[2])
			, GetParticleAt(points[1]), GetParticleAt(points[0]), stiff);
		TATPBDWorld::Instance()->AddConstraint(constr);
	}
}

bool TATPBDBody::Update(TATActor* actor, float dt)
{
	return true;
}

void TATPBDBody::StepSimulation(float dt)
{
	for (int i = 0; i < m_Particles.size(); ++i)
	{
		TATPBDParticle& particle = m_Particles[i];
		particle.m_ExternalForce = particle.m_InstantForce + particle.m_ConstantForce;
		particle.m_InstantForce.SetZero();
		particle.m_Velocity += particle.m_ExternalForce * particle.m_InvMass * dt;
		DampVelocity(particle, dt);

		particle.m_PredictPos = particle.Position() + dt * particle.m_Velocity;
	}

	//generate collision

	//v <- v + dt * im * f
	//damp v
	//pi <- xi + dt * v

	//collision xi->pi

	//constraint solve

	//vi <- (pi - xi)/dt;
	//xi <- pi
	//velocityUpdate

}

void TATPBDBody::DampVelocity(TATPBDParticle& particle, float dt)
{
	particle.m_Velocity *= (1 - m_LinearDamp * dt);
}

void TATPBDBody::Integrate(float dt)
{
	for (int i = 0; i < m_Particles.size(); ++i)
	{
		TATPBDParticle& particle = m_Particles[i];
		particle.m_Velocity = (particle.m_PredictPos - particle.Position()) / dt;
		particle.Position() = particle.m_PredictPos;
		
		//friction and restitution adjust velocity
	}
}

void TATPBDBody::UpdateAabb()
{
	m_ParticleBVH.Clear();
	for (int i = 0; i < m_Particles.size(); ++i)
	{
		TATPBDParticle& particle = m_Particles[i];
		TATAabb bound;
		bound.SetOrigin(particle.Position(), particle.m_PredictPos);
		TATBVNode* node = m_ParticleBVH.InsertAabbNode(bound.m_OriginMin - TATVector3::One() * 0.1f,
			bound.m_OriginMax + TATVector3::One() * 0.1f);
		node->m_Data = &m_Particles[i];
	}
	m_ParticleBVH.FinishBuild();
}

void TATPBDBody::SolveConstraintEnd()
{
	std::vector<TATVector3> points;
	points.resize(m_RenderParticles.size());
	for (int i = 0; i < points.size(); ++i)
	{
		points[i] = m_PhyBody.m_Vertices[m_RenderParticles[i]].m_Position;
	}

	if (m_HostActor)
	{
		TAT_RENDER_TASK_LIST->PushTask([](std::vector<TATVector3> points, TATActor* actor)->void
		{
			int sz = points.size();
			for (int i = 0; i < sz; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					actor->m_RenderMesh->m_MeshVertices[i].m_Position[j] = points[i][j];
				}

			}

			actor->MarkRenderStateDirty();

		}, points, m_HostActor);
	}

}