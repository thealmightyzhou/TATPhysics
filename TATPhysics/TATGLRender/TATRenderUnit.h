#pragma once
#include "TATGLHeader.h"
#include "../TATCommon/TATCore.h"
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATransform.h"
#include "../TATResources/TATModelLoader.h"

struct TATRenderVertex
{
public:
	TATVector3 m_Position;
	TATVector3 m_Normal;
	TATVector3 m_Tangent;
	float m_TexCoordinates[MAX_TEXCOORDINATE_COUNT][2];
	int m_TexCoordinateCount;
};

class TATRenderUnit
{
public:
	TATRenderUnit()
	{
		m_RenderBuffer = 0;
		m_VertexOrder = 0;
		m_MatrixView = 0;
		m_MatrixProj = 0;
		m_MatrixModel = 0;
		m_RenderVertices = 0;
		m_IndicesCount = 0;
		m_VertexCount = 0;
		m_VAOId = -1;
		m_VBOId = -1;
		m_StaticDataUploaded = false;
	}

	TATShader* m_Shader;
	TATMaterial* m_Material;

	int m_VertexCount;
	int m_TriangleCount;
	int m_TexCoordinateCount;
	int m_CameraCount;
	int m_IndicesCount;
	int m_TexCount;

	float* m_RenderBuffer; //a basic unit to render include vertex,normal,uv,tangent which transport to glbuffer
	int* m_VertexOrder; //indices can be divided by 3;

	float* m_MatrixView;
	float* m_MatrixProj;
	float* m_MatrixModel;

	TATRenderVertex* m_RenderVertices; //already sort by face's vertex indices

	TATexture** m_Textures;
	UINT* m_TextureIds;
	UINT m_ShaderId;
	UINT m_VAOId;
	UINT m_VBOId;

	TATransform *m_TransformPtr;

	bool m_UseTransform; //transform to wcs or already in wcs

	TATModelElementMask m_RenderEleMask;

	bool m_StaticDataUploaded;

	void SetMaterial(TATMaterial* mat)
	{
		m_Material = mat;
		mat->OnMaterialSetted(this);
	}

	void GenerateRenderBuffer()
	{
		if (m_VertexCount == 0)
			return;

		//temp use point render
		if (m_IndicesCount == 0)
		{
			m_VertexOrder = new int[m_VertexCount];

			for (int i = 0; i < m_VertexCount; i++)
			{
				m_VertexOrder[i] = i;
			}
		}

		int blockSize = m_RenderEleMask.ComputeSize();
		m_RenderBuffer = new float[blockSize * m_IndicesCount];
		int curr = 0;
		for (int i = 0; i < blockSize * m_IndicesCount; i += blockSize)
		{
			int index = i / blockSize;
			const TATRenderVertex& vertex = m_RenderVertices[m_VertexOrder[index]];

			m_RenderBuffer[i * blockSize + curr++] = vertex.m_Position[0];
			m_RenderBuffer[i * blockSize + curr++] = vertex.m_Position[1];
			m_RenderBuffer[i * blockSize + curr++] = vertex.m_Position[2];

			if (m_RenderEleMask.IsUseNormal())
			{
				m_RenderBuffer[i * blockSize + curr++] = vertex.m_Normal[0];
				m_RenderBuffer[i * blockSize + curr++] = vertex.m_Normal[1];
				m_RenderBuffer[i * blockSize + curr++] = vertex.m_Normal[2];
			}

			if (m_RenderEleMask.IsUseTangent())
			{
				m_RenderBuffer[i * blockSize + curr++] = vertex.m_Tangent[0];
				m_RenderBuffer[i * blockSize + curr++] = vertex.m_Tangent[1];
				m_RenderBuffer[i * blockSize + curr++] = vertex.m_Tangent[2];
			}

			if (m_RenderEleMask.IsUseTexCoordinate())
			{
				for (int t = 0; t < m_RenderEleMask.m_TexCount; t++)
				{
					m_RenderBuffer[i * blockSize + curr++] = vertex.m_TexCoordinates[t][0];
					m_RenderBuffer[i * blockSize + curr++] = vertex.m_TexCoordinates[t][1];
				}
			}

			assert(curr == blockSize);

			curr = 0;
		}
	}
};