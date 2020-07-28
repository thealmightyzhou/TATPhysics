#include "TATRenderUnit.h"
#include "../TATResources/TATMaterial.h"
#include "../TATCommon/TATCore.h"
#include "TATGLHeader.h"

TATRenderUnit::TATRenderUnit()
{
	m_RenderBuffer = 0;
	m_VertexOrder = 0;
	m_RenderVertices = 0;
	m_IndicesCount = 0;
	m_VertexCount = 0;
	m_VAOId = TAT_SHADERID_UNUSE;
	m_VBOId = TAT_SHADERID_UNUSE;
	m_StaticDataUploaded = false;
	TATMatrix3::GetIdentity().GetOpenGLSubMatrix(m_MatrixModel);
	TATMatrix3::GetIdentity().GetOpenGLSubMatrix(m_MatrixProj);
	TATMatrix3::GetIdentity().GetOpenGLSubMatrix(m_MatrixView);
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
		TAT_SAFE_NEW(m_VertexOrder, int, m_VertexCount);

		for (int i = 0; i < m_VertexCount; i++)
		{
			m_VertexOrder[i] = i;
		}
	}

	int blockSize = m_RenderEleMask.ComputeSize();

	TAT_SAFE_NEW(m_RenderBuffer, float, blockSize * m_IndicesCount);

	int curr = 0;
	for (int i = 0; i < blockSize * m_IndicesCount; i += blockSize)
	{
		int index = i / blockSize;
		const TATRenderVertex& vertex = m_RenderVertices[m_VertexOrder[index]];

		m_RenderBuffer[i + curr++] = vertex.m_Position[0];
		m_RenderBuffer[i + curr++] = vertex.m_Position[1];
		m_RenderBuffer[i + curr++] = vertex.m_Position[2];

		if (m_RenderEleMask.IsUseNormal())
		{
			m_RenderBuffer[i + curr++] = vertex.m_Normal[0];
			m_RenderBuffer[i + curr++] = vertex.m_Normal[1];
			m_RenderBuffer[i + curr++] = vertex.m_Normal[2];
		}

		if (m_RenderEleMask.IsUseTangent())
		{
			m_RenderBuffer[i + curr++] = vertex.m_Tangent[0];
			m_RenderBuffer[i + curr++] = vertex.m_Tangent[1];
			m_RenderBuffer[i + curr++] = vertex.m_Tangent[2];
		}

		if (m_RenderEleMask.IsUseTexCoordinate())
		{
			for (int t = 0; t < m_RenderEleMask.m_TexCount; t++)
			{
				m_RenderBuffer[i + curr++] = vertex.m_TexCoordinates[t][0];
				m_RenderBuffer[i + curr++] = vertex.m_TexCoordinates[t][1];
			}
		}

		assert(curr == blockSize);

		curr = 0;
	}

	for (int i = 0; i < MAX_TEXCOORDINATE_COUNT; ++i)
	{
		if (m_Material->m_Textures[i])
		{
			m_TextureIds[i] = m_Material->m_Textures[i]->GetGLId();
		}
	}
}

void TATRenderUnit::Clear()
{
	if (m_RenderBuffer)
	{
		delete m_RenderBuffer;
		m_RenderBuffer = 0;
	}
	if (m_VertexOrder)
	{
		delete m_VertexOrder;
		m_VertexOrder = 0;
	}
	if (m_RenderVertices)
	{
		delete m_RenderVertices;
		m_RenderVertices = 0;
	}

	m_IndicesCount = 0;
	m_VertexCount = 0;
	m_TexCoordinateCount = 0;
	m_TexCount = 0;
	m_Material = 0;
	m_ReadyToRender = false;
	m_ShaderId = -1;
	m_StaticDataUploaded = false;
	m_TriangleCount = 0;
	m_UseTransform = false;
	m_StaticDataUploaded = false;
	TAT_MEMSET(m_TextureIds, TAT_SHADERID_UNUSE);
	TAT_MEMSET(m_Textures, 0);
	TATMatrix3::GetIdentity().GetOpenGLSubMatrix(m_MatrixModel);
	TATMatrix3::GetIdentity().GetOpenGLSubMatrix(m_MatrixProj);
	TATMatrix3::GetIdentity().GetOpenGLSubMatrix(m_MatrixView);
	m_RenderEleMask.Clear();
	if(m_VAOId != TAT_SHADERID_UNUSE)
		glDeleteVertexArrays(1, &m_VAOId);
	if (m_VBOId != TAT_SHADERID_UNUSE)
		glDeleteBuffers(1, &m_VBOId);

	m_VAOId = TAT_SHADERID_UNUSE;
	m_VBOId = TAT_SHADERID_UNUSE;
}