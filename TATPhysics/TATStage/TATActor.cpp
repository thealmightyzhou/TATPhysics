#include "TATActor.h"
#include "../TATApplication/TAThread.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"
#include "../TATBasis/TATWorld.h"
#include "../TATResources/TATMesh.h"

TATActor::TATActor(TATMesh* ptr) :TATObject("actor_" + ptr->GetSubName() + TString::Make(GetObjectIndex()))
{
	m_RenderMesh = ptr;
	m_RenderCamera = TATWorld::Instance()->GetCamera("main");
	m_RenderLight = TATWorld::Instance()->GetLight("main");
	m_RenderUnit = TATRenderThread::Instance()->m_RenderUnitPool.FetchUnused();
	m_RigidBodyId = -1;
}

TATActor::~TATActor()
{
	m_RenderMesh = 0;
	m_RenderCamera = 0;
	m_RenderLight = 0;
	TATRenderThread::Instance()->m_RenderUnitPool.ReturnUsed(m_RenderUnit);
	m_RenderUnit = 0;
	m_RigidBodyId = -1;
}

void TATActor::FillRenderUnit()
{
	if (!m_RenderMesh)
		return;

	TATModelLoader* loader = m_RenderMesh->m_Loader;

	m_RenderUnit->m_RenderEleMask = loader->m_ModelElementMask;

	m_RenderCamera->GetViewMatrix(m_RenderUnit->m_MatrixView);
	m_RenderCamera->GetProjectionMatrix(m_RenderUnit->m_MatrixProj);
	m_WorldTransform.GetOpenGLMatrix(m_RenderUnit->m_MatrixModel);

	m_RenderUnit->m_VertexCount = m_RenderMesh->m_VertexCount;
	if (!m_RenderUnit->m_RenderVertices)
		m_RenderUnit->m_RenderVertices = new TATRenderVertex[m_RenderUnit->m_VertexCount];
	TATRenderVertex* vertices = m_RenderUnit->m_RenderVertices;
	for (int i = 0; i < m_RenderUnit->m_VertexCount; i++)
	{
		vertices[i].m_Position = m_RenderMesh->m_MeshVertices[i].m_Position;
		if (m_RenderUnit->m_RenderEleMask.IsUseNormal())
		{
			vertices[i].m_Normal = m_RenderMesh->m_MeshVertices[i].m_Normal;
		}

		if (m_RenderUnit->m_RenderEleMask.IsUseTangent())
		{
			vertices[i].m_Tangent = m_RenderMesh->m_MeshVertices[i].m_Tangent;
		}

		if (m_RenderUnit->m_RenderEleMask.IsUseTexCoordinate())
		{
			vertices[i].m_TexCoordinateCount = loader->m_ModelElementMask.m_TexCount;
			for (int u = 0; u < vertices[i].m_TexCoordinateCount; u++)
			{
				vertices[i].m_TexCoordinates[u][0] = loader->m_Buffer.vertexBuffer[i].m_TexCoordinates[u][0];
				vertices[i].m_TexCoordinates[u][1] = loader->m_Buffer.vertexBuffer[i].m_TexCoordinates[u][1];
			}
		}

	}

	m_RenderUnit->m_IndicesCount = 3 * m_RenderMesh->m_FaceCount;
	if (!m_RenderUnit->m_VertexOrder && m_RenderUnit->m_IndicesCount != 0)
		m_RenderUnit->m_VertexOrder = new int[m_RenderUnit->m_IndicesCount];
	for (int i = 0; i < m_RenderUnit->m_IndicesCount; i += 3)
	{
		m_RenderUnit->m_VertexOrder[i + 0] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[0];
		m_RenderUnit->m_VertexOrder[i + 1] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[1];
		m_RenderUnit->m_VertexOrder[i + 2] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[2];
	}

	m_RenderUnit->GenerateRenderBuffer();

	m_RenderUnit->m_ReadyToRender = true;
}