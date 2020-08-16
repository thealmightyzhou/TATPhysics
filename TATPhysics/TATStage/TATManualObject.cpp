#include "TATManualObject.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATResources/TATMaterial.h"

TATManualObject::TATManualObject()
{
	m_RenderEleMask.Clear();
	m_RenderMode = TAT_RENDERMODE_LINES;
}

void TATManualObject::FillRenderUnit()
{
	//upload static data once is enough
	if (!m_RenderStateDirty)
		return;

	m_RenderUnit->Clear();

	m_RenderUnit->m_RenderEleMask = m_RenderEleMask;
	m_RenderUnit->m_RenderMode = m_RenderMode;
	m_RenderCamera->GetViewMatrix(m_RenderUnit->m_MatrixView);
	m_RenderCamera->GetProjectionMatrix(m_RenderUnit->m_MatrixProj);
	m_WorldTransform.GetOpenGLMatrix(m_RenderUnit->m_MatrixModel);

	m_RenderUnit->m_VertexCount = m_RenderVertices.size();

	TAT_SAFE_NEW(m_RenderUnit->m_RenderVertices, TATRenderVertex, m_RenderUnit->m_VertexCount);

	TATRenderVertex* vertices = m_RenderUnit->m_RenderVertices;
	for (int i = 0; i < m_RenderUnit->m_VertexCount; i++)
	{
		vertices[i].m_Position = m_RenderVertices[i].m_Position;
		if (m_RenderUnit->m_RenderEleMask.IsUseNormal())
		{
			vertices[i].m_Normal = m_RenderVertices[i].m_Normal;
		}

		if (m_RenderUnit->m_RenderEleMask.IsUseTangent())
		{
			vertices[i].m_Tangent = m_RenderVertices[i].m_Tangent;
		}

		if (m_RenderUnit->m_RenderEleMask.IsUseColour())
		{
			vertices[i].m_Colour = m_RenderVertices[i].m_Colour;
		}

		if (m_RenderUnit->m_RenderEleMask.IsUseTexCoordinate())
		{
			vertices[i].m_TexCoordinateCount = m_RenderUnit->m_RenderEleMask.m_TexCount;
			for (int u = 0; u < vertices[i].m_TexCoordinateCount; u++)
			{
				vertices[i].m_TexCoordinates[u][0] = m_RenderVertices[i].m_TexCoordinates[u][0];
				vertices[i].m_TexCoordinates[u][1] = m_RenderVertices[i].m_TexCoordinates[u][1];
			}
		}
	}

	m_RenderUnit->m_IndicesCount = m_RenderOrder.size();

	TAT_SAFE_NEW(m_RenderUnit->m_VertexOrder, int, m_RenderUnit->m_IndicesCount);

	for (int i = 0; i < m_RenderUnit->m_IndicesCount; ++i)
	{
		m_RenderUnit->m_VertexOrder[i] = m_RenderOrder[i];
	}

	m_RenderUnit->m_TexCoordinateCount = m_RenderEleMask.m_TexCount;
	m_RenderUnit->m_TexCount = m_RenderUnit->m_TexCoordinateCount;
	TAT_MEMCPY(m_RenderUnit->m_Textures, m_Material->m_Textures);

	m_RenderUnit->m_Material = m_Material;
	m_RenderUnit->GenerateRenderBuffer();

	m_RenderUnit->m_ReadyToRender = true;
	m_RenderStateDirty = false;
}