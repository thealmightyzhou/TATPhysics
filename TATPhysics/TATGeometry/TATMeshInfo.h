#pragma once
#include "TATPhyPrimitive.h"
#include "../TATCommon/TATMatrix3.h"
#include <unordered_set>
#include "../TATCommon/TATAabb.h"

struct TATPhyMeshData
{
	std::vector<TATPhyVertex>	m_Vertices;
	std::vector<TATPhyFace>		m_Faces;
	std::vector<TATPhyEdge>		m_Edges;
	TATVector3					m_MassCentre;
	TATAabb						m_LocalAabb;
};

class TATPhyMeshDataComputer
{
public:
	TATVector3 static ComputeMassCentreAndLocalAabb(const std::vector<TATPhyVertex>& vertices,TATVector3& omin,TATVector3& omax)
	{
		TATVector3 min = TAT_MAXVECTOR3;
		TATVector3 max = -TAT_MAXVECTOR3;
		for (int i = 0; i < (int)vertices.size(); i++)
		{
			min.SetMin(vertices[i].GetPosition());
			max.SetMax(vertices[i].GetPosition());
		}
		omin = min;
		omax = max;

		return (min + max) / 2;
	}

	TATMatrix3 static ComputeInertiaTensor(const std::vector<TATPhyVertex>& vertices,const TATVector3& centre,float mass)
	{
		float perMass = mass / vertices.size();
		TATMatrix3 inertia;
		for (int i = 0; i < (int)vertices.size(); i++)
		{
			TATVector3 r = vertices[i].GetPosition() - centre;

			inertia(0, 0) += (r.Y * r.Y + r.Z * r.Z) * perMass;

			inertia(1, 1) += (r.X * r.X + r.Z * r.Z) * perMass;

			inertia(2, 2) += (r.X * r.X + r.Y * r.Y) * perMass;

			inertia(0, 1) += r.X * r.Y * perMass;

			inertia(0, 2) += r.X * r.Z * perMass;

			inertia(1, 2) += r.Y * r.Z * perMass;
		}

		inertia(1, 0) = inertia(0, 1);

		inertia(2, 0) = inertia(0, 2);

		inertia(2, 1) = inertia(1, 2);

		return inertia;
	}

	float static ComputeInertiaDiag(const TATMatrix3& inertia)
	{
		return inertia[0][0] * inertia[1][1] * inertia[2][2];
	}

	void static CompleteMeshData(std::vector<TATPhyVertex>& vertices, std::vector<TATPhyFace>& faces, std::vector<TATPhyEdge>& edges)
	{
		CompleteFaceAndVertex(vertices, faces); //face has vertex info and so as vertex

		if (edges.size() == 0)
		{
			CreateEdges(vertices, faces, edges); //edge has vertex and face info
		}

	}

	void static CompleteFaceAndVertex(std::vector<TATPhyVertex>& vertices, std::vector<TATPhyFace>& faces)
	{
		if (faces.size() == 0)
			return;
		bool res;
		for (int f = 0; f < (int)faces.size(); f++)
		{
			TATPhyFace& face = faces[f];
			TAT_MEMEMPTY(face.m_Vertices, 0, res); //default fill the indices
			if (res)
			{
				face.m_Vertices[0] = &vertices[face.m_VertexIndices[0]];
				face.m_Vertices[1] = &vertices[face.m_VertexIndices[1]];
				face.m_Vertices[2] = &vertices[face.m_VertexIndices[2]];

				face.m_Vertices[0]->m_FaceIndices.push_back(f);
				face.m_Vertices[1]->m_FaceIndices.push_back(f);
				face.m_Vertices[2]->m_FaceIndices.push_back(f);

				face.m_Vertices[0]->m_Faces.push_back(&faces[f]);
				face.m_Vertices[1]->m_Faces.push_back(&faces[f]);
				face.m_Vertices[2]->m_Faces.push_back(&faces[f]);

				face.m_Normal = ((face.m_Vertices[1]->m_Position - face.m_Vertices[0]->m_Position)
					.Cross(face.m_Vertices[2]->m_Position - face.m_Vertices[0]->m_Position)).Normalized();
			}
		}
	}

	typedef std::unordered_set<TATPhyEdge, TATPhyEdgeHasher> TEdgeSet;

	void static CreateEdges(std::vector<TATPhyVertex>& vertices, std::vector<TATPhyFace>& faces, std::vector<TATPhyEdge>& edges)
	{
		TEdgeSet edgeSet;
		TATPhyEdgeHasher hasher;
		for (int f = 0; f < (int)faces.size(); f++)
		{
			TATPhyFace& face = faces[f];
			TATPhyEdge tmpEdges[3]{
				TATPhyEdge(face.m_VertexIndices[0], face.m_VertexIndices[1]),
				TATPhyEdge(face.m_VertexIndices[1], face.m_VertexIndices[2]),
				TATPhyEdge(face.m_VertexIndices[2], face.m_VertexIndices[0]), };

			tmpEdges[0].m_HashMask = hasher(tmpEdges[0]);
			tmpEdges[1].m_HashMask = hasher(tmpEdges[1]);
			tmpEdges[2].m_HashMask = hasher(tmpEdges[2]);

			TEdgeSet::iterator it;
			for (int i = 0; i < 3; i++)
			{
				TEdgeSet::iterator it = edgeSet.find(tmpEdges[i]);
				if (it == edgeSet.end())
				{
					tmpEdges[i].m_FaceIndices[0] = f;
					edgeSet.insert(tmpEdges[i]);
				}
				else
				{
					TATPhyEdge tempe = *it;
					tempe.m_FaceIndices[1] = f;
					edgeSet.erase(it);
					edgeSet.insert(tempe);
				}
			}
		}

		TEdgeSet::iterator it = edgeSet.begin();
		int index = 0;
		edges.resize(edgeSet.size());
		while (it!= edgeSet.end())
		{
			edges[index] = *it;
			for (int i = 0; i < 2; i++)
			{
				edges[index].m_Faces[i] = &faces[edges[index].m_FaceIndices[i]];
				edges[index].m_Vertices[i] = &vertices[edges[index].m_VertexIndices[i]];
			}
			edges[index].m_Direction = (edges[index].m_Vertices[1]->m_Position - edges[index].m_Vertices[0]->m_Position).Normalized();

			it++;
			index++;
		}
	}
};
