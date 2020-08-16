#pragma once

#include "../TATCommon/TATVector3.h"

struct TATPhyFace;
struct TATPhyEdge;

struct TATPhyVertex
{
public:
	TATPhyVertex()
	{
		m_Position = TATVector3::Zero();
		Init();
	}

	TATPhyVertex(const TATVector3& v) :m_Position(v)
	{
		Init();
	}

	void Init()
	{
	}

	TAT_REGISTER_ATTRIBUTE(TATVector3, Position);

	bool operator==(const TATPhyVertex& v)
	{
		return (m_Position.Distance(v.m_Position) < TAT_EPSILON2);
	}

	std::vector<UINT> m_FaceIndices;
	std::vector<UINT> m_EdgeIndices;
	std::vector<TATPhyFace*> m_Faces;
	std::vector<TATPhyEdge*> m_Edges;
};

struct TATPhyFace
{
public:
	TATPhyFace() 
	{
		Init();
	}

	TATPhyFace(TATPhyVertex* v0, TATPhyVertex* v1, TATPhyVertex* v2)
	{
		Init();
		m_Vertices[0] = v0;
		m_Vertices[1] = v1;
		m_Vertices[2] = v2;
	}

	TATPhyFace(UINT v0, UINT v1, UINT v2)
	{
		Init();
		m_VertexIndices[0] = v0;
		m_VertexIndices[1] = v1;
		m_VertexIndices[2] = v2;
	}

	void Init()
	{
		TAT_MEMSET(m_VertexIndices, 0);
		TAT_MEMSET(m_EdgeIndices, 0);
		TAT_MEMSET(m_Vertices, 0);
		TAT_MEMSET(m_Edges, 0);
	}

	bool operator==(const TATPhyFace& f)
	{
		bool res;
		TAT_ARRAYCONTAIN(m_VertexIndices, f.m_VertexIndices, 3, 3, res);
		return res;
	}

	UINT			m_VertexIndices[3];
	UINT			m_EdgeIndices[3];
	TATPhyVertex*	m_Vertices[3];
	TATPhyEdge*		m_Edges[3];
	TAT_REGISTER_ATTRIBUTE(TATVector3, Normal);

};

struct TATPhyEdge
{
public:
	TATPhyEdge()
	{
		Init();
	}

	TATPhyEdge(TATPhyVertex* v0, TATPhyVertex* v1)
	{
		m_Vertices[0] = v0;
		m_Vertices[1] = v1;
	}

	TATPhyEdge(UINT v0, UINT v1)
	{
		m_VertexIndices[0] = v0;
		m_VertexIndices[1] = v1;
	}

	void Init()
	{
		TAT_MEMSET(m_VertexIndices, 0);
		TAT_MEMSET(m_FaceIndices, 0);
		TAT_MEMSET(m_Vertices, 0);
		TAT_MEMSET(m_Faces, 0);
	}

	bool operator==(const TATPhyEdge& e) const
	{
		if (m_VertexIndices[0] == e.m_VertexIndices[0] && m_VertexIndices[1] == e.m_VertexIndices[1] ||
			m_VertexIndices[0] == e.m_VertexIndices[1] && m_VertexIndices[1] == e.m_VertexIndices[0])
		{
			return true;
		}

		return false;
	}

	bool operator<(const TATPhyEdge& e) const
	{
		return !(*this == e);
	}

	TATVector3 GetHeadPosition()
	{
		return m_Vertices[0]->GetPosition();
	}

	TATVector3 GetTailPosition()
	{
		return m_Vertices[1]->GetPosition();
	}

	UINT			m_VertexIndices[2];
	UINT			m_FaceIndices[2];
	TATPhyVertex*	m_Vertices[2];
	TATPhyFace*		m_Faces[2];
	TAT_REGISTER_ATTRIBUTE(TATVector3, Direction);

	UINT			m_HashMask;
};

struct TATPhyEdgeHasher
{
	size_t operator()(const TATPhyEdge& e) const
	{
		std::hash<int> hasher;
		size_t seed = 0;

		int xy = e.m_VertexIndices[0] * e.m_VertexIndices[1];
		int x_y = e.m_VertexIndices[0] + e.m_VertexIndices[1];

		seed ^= hasher(xy) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
		seed ^= hasher(x_y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);

		return seed;
	}
};


struct TATPhyTetra
{
public:
	TATPhyTetra()
	{
		Init();
	}

	TATPhyTetra(TATPhyVertex* v0, TATPhyVertex* v1, TATPhyVertex* v2, TATPhyVertex* v3)
	{
		Init();
		m_Vertices[0] = v0;
		m_Vertices[1] = v1;
		m_Vertices[2] = v2;
		m_Vertices[3] = v3;
	}

	TATPhyTetra(UINT v0, UINT v1, UINT v2,UINT v3)
	{
		Init();
		m_VertexIndices[0] = v0;
		m_VertexIndices[1] = v1;
		m_VertexIndices[2] = v2;
		m_VertexIndices[3] = v3;
	}

	void Init()
	{
		TAT_MEMSET(m_VertexIndices, 0);
		TAT_MEMSET(m_EdgeIndices, 0);
		TAT_MEMSET(m_Vertices, 0);
		TAT_MEMSET(m_Edges, 0);
	}

	bool operator==(const TATPhyTetra& f)
	{
		bool res;
		TAT_ARRAYCONTAIN(m_VertexIndices, f.m_VertexIndices, 4, 4, res);
		return res;
	}

	UINT			m_VertexIndices[4];
	UINT			m_EdgeIndices[6];
	TATPhyVertex*	m_Vertices[4];
	TATPhyEdge*		m_Edges[6];

};