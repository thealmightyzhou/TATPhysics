#pragma once
#include "../TATCommon/TATVector3.h"
#include "TATResourcePrimitive.h"
#include "TATModelLoader.h"
#include "TATObjLoader.h"
#include "../TATBasis/TString.h"

struct TATRendVertex
{
public:
	float m_Position[3];
	float m_Normal[3];
	float m_TexCoordinate[2];
	float m_Tangent[3];
};

struct TATRendFace
{
public:
	int m_Vertices[3];
};

class TATMesh :public TATResourcePrimitive
{
public:

	TATMesh():TATResourcePrimitive("mesh_nameless" + GetObjectIndex())
	{
		m_RendVertices = 0;
		m_RendFaces = 0;
	}

	TATMesh(const TString& name):TATResourcePrimitive("mesh_" + name)
	{
		m_RendVertices = 0;
		m_RendFaces = 0;
	}

	~TATMesh()
	{
		Clear();
		delete m_Loader;
	}

	virtual void Load(const TString& name)
	{
		__super::Load(name);

		Clear();

		std::vector<TString> strs = name.Split(".");

		if (strs[1] == "obj")
		{
			m_Loader = new TATObjLoader(name);
		}
		else if (strs[1] == "mms")
		{
			//TODO
		}

		if (!m_Loader)
			return;

		const TATModelBuffer& buffer = m_Loader->m_Buffer;
		
		m_VertexCount = buffer.vertexBuffer.size();
		m_RendVertices = new TATRendVertex[m_VertexCount];
		for (int i = 0; i < m_VertexCount; ++i)
		{
			m_RendVertices[i].m_Position[0] = buffer.vertexBuffer[i].x;
			m_RendVertices[i].m_Position[1] = buffer.vertexBuffer[i].y;
			m_RendVertices[i].m_Position[2] = buffer.vertexBuffer[i].z;

			m_RendVertices[i].m_Normal[0] = buffer.vertexBuffer[i].nx;
			m_RendVertices[i].m_Normal[1] = buffer.vertexBuffer[i].ny;
			m_RendVertices[i].m_Normal[2] = buffer.vertexBuffer[i].nz;

			m_RendVertices[i].m_TexCoordinate[0] = buffer.vertexBuffer[i].u;
			m_RendVertices[i].m_TexCoordinate[1] = buffer.vertexBuffer[i].v;
		}

		m_FaceCount = buffer.faceBuffer.size();
		m_RendFaces = new TATRendFace[m_FaceCount];
		for (int i = 0; i < m_FaceCount; ++i)
		{
			m_RendFaces[i].m_Vertices[0] = buffer.faceBuffer[i].v1;
			m_RendFaces[i].m_Vertices[1] = buffer.faceBuffer[i].v2;
			m_RendFaces[i].m_Vertices[2] = buffer.faceBuffer[i].v3;
		}
	}

	void Clear()
	{
		if (m_RendVertices)
		{
			delete m_RendVertices;
			m_RendVertices = 0;
		}

		if (m_RendFaces)
		{
			delete m_RendFaces;
			m_RendFaces = 0;
		}

	}

	int m_VertexCount;
	int m_FaceCount;

	TATRendVertex* m_RendVertices;
	TATRendFace* m_RendFaces;

	TATModelLoader* m_Loader;

};