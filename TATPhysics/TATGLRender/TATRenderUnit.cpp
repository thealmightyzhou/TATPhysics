#include "TATRenderUnit.h"
#include "../TATResources/TATMaterial.h"

TATRenderUnit::TATRenderUnit()
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

void TATRenderUnit::SetMaterial(TATMaterial* mat)
{
	m_Material = mat;
	mat->OnMaterialSetted(this);
}

void TATRenderUnit::GenerateRenderBuffer()
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