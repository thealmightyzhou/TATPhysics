#pragma once
#include "../TATCommon/TATVector3.h"
#include "TATResourcePrimitive.h"
#include "TATObjLoader.h"
#include "../TATBasis/TString.h"

struct TATMeshVertex
{
public:
	float m_Position[3];
	float m_Normal[3];
	float m_TexCoordinate[MAX_TEXCOORDINATE_COUNT][2];
	float m_Tangent[3];
};

struct TATMeshFace
{
public:
	int m_Vertices[3];
};

class TATMesh :public TATResourcePrimitive
{
public:

	TATMesh():TATResourcePrimitive("mesh_nameless" + GetObjectIndex())
	{
		m_MeshVertices = 0;
		m_MeshFaces = 0;
		m_VertexCount = 0;
		m_FaceCount = 0;
	}

	TATMesh(const TString& name):TATResourcePrimitive("mesh_" + name)
	{
		m_MeshVertices = 0;
		m_MeshFaces = 0;
		m_VertexCount = 0;
		m_FaceCount = 0;
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

		std::vector<TString> strs;
		name.Split(".", strs);

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
		m_ModelElementMask = m_Loader->m_ModelElementMask;
		
		m_VertexCount = buffer.vertexBuffer.size();
		m_MeshVertices = new TATMeshVertex[m_VertexCount];
		for (int i = 0; i < m_VertexCount; ++i)
		{
			for (int c = 0; c < 3; c++)
			{
				m_MeshVertices[i].m_Position[c] = buffer.vertexBuffer[i].m_Position[c];
				m_MeshVertices[i].m_Normal[c] = buffer.vertexBuffer[i].m_Normal[c];
			}

			m_MeshVertices[i].m_TexCoordinate[0][0] = buffer.vertexBuffer[i].m_TexCoordinates[0][0];
			m_MeshVertices[i].m_TexCoordinate[0][1] = buffer.vertexBuffer[i].m_TexCoordinates[0][1];
		}

		m_FaceCount = buffer.faceBuffer.size();
		if (m_FaceCount > 0)
		{
			m_MeshFaces = new TATMeshFace[m_FaceCount];
			for (int i = 0; i < m_FaceCount; ++i)
			{
				m_MeshFaces[i].m_Vertices[0] = buffer.faceBuffer[i].v1;
				m_MeshFaces[i].m_Vertices[1] = buffer.faceBuffer[i].v2;
				m_MeshFaces[i].m_Vertices[2] = buffer.faceBuffer[i].v3;
			}
		}

	}

	void Clear()
	{
		if (m_MeshVertices)
		{
			delete m_MeshVertices;
			m_MeshVertices = 0;
			m_VertexCount = 0;
		}

		if (m_MeshFaces)
		{
			delete m_MeshFaces;
			m_MeshFaces = 0;
			m_FaceCount = 0;
		}

	}

	int m_VertexCount;
	int m_FaceCount;

	TATMeshVertex* m_MeshVertices;
	TATMeshFace* m_MeshFaces;

	TATModelLoader* m_Loader;

	TATModelElementMask m_ModelElementMask;
};