#pragma once
#include "../TATCommon/TATCore.h"
#include "../TATBasis/TATObject.h"
#include "../TATResources/TATMesh.h"
#include "../TATCommon/TATransform.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"

//base object to show on stage
class TATActor :public TATObject
{
public:
	TATActor(TATResourcePtr<TATMesh> ptr) :TATObject("actor_" + ptr->GetSubName() + TString::Make(GetObjectIndex()))
	{
		m_RenderMesh = &(*ptr);
	}

	void FillRenderUnit()
	{
		if (!m_RenderMesh)
			return;

		TATModelLoader* loader = m_RenderMesh->m_Loader;

		m_RenderUnit.m_RenderEleMask = loader->m_ModelElementMask;

		m_RenderUnit.m_CameraCount = m_RenderCameras.size();
		if(!m_RenderUnit.m_MatrixViews)
			m_RenderUnit.m_MatrixViews = new float* [m_RenderUnit.m_CameraCount];
		if(!m_RenderUnit.m_MatrixProjs)
			m_RenderUnit.m_MatrixProjs = new float* [m_RenderUnit.m_CameraCount];
		for (int i = 0; i < m_RenderUnit.m_CameraCount; i++)
		{
			m_RenderUnit.m_MatrixViews[i] = new float[16];
			m_RenderUnit.m_MatrixProjs[i] = new float[16];
			m_RenderCameras[i]->GetViewMatrix(m_RenderUnit.m_MatrixViews[i]);
			m_RenderCameras[i]->GetProjectionMatrix(m_RenderUnit.m_MatrixProjs[i]);
		}

		if(!m_RenderUnit.m_MatrixModel)
			m_RenderUnit.m_MatrixModel = new float[16];
		m_WorldTransform.GetOpenGLMatrix(m_RenderUnit.m_MatrixModel);

		m_RenderUnit.m_VertexCount = m_RenderMesh->m_VertexCount;
		if (!m_RenderUnit.m_RenderVertices)
			m_RenderUnit.m_RenderVertices = new TATRenderVertex[m_RenderUnit.m_VertexCount];
		TATRenderVertex* vertices = m_RenderUnit.m_RenderVertices;
		for (int i = 0; i < m_RenderUnit.m_VertexCount; i++)
		{
			vertices[i].m_Position = m_RenderMesh->m_MeshVertices[i].m_Position;
			if (m_RenderUnit.m_RenderEleMask.IsUseNormal())
			{
				vertices[i].m_Normal = m_RenderMesh->m_MeshVertices[i].m_Normal;
			}

			if (m_RenderUnit.m_RenderEleMask.IsUseTangent())
			{
				//TODO
			}

			if (m_RenderUnit.m_RenderEleMask.IsUseTexCoordinate())
			{
				vertices[i].m_TexCoordinateCount = loader->m_ModelElementMask.m_TexCount;
				for (int u = 0; u < vertices[i].m_TexCoordinateCount; u++)
				{
					vertices[i].m_TexCoordinates[u][0] = loader->m_Buffer.vertexBuffer[i].m_TexCoordinates[u][0];
					vertices[i].m_TexCoordinates[u][1] = loader->m_Buffer.vertexBuffer[i].m_TexCoordinates[u][1];
				}
			}

		}

		m_RenderUnit.m_IndicesCount = 3 * m_RenderMesh->m_FaceCount;
		m_RenderUnit.m_VertexOrder = new int[m_RenderUnit.m_IndicesCount];
		for (int i = 0; i < m_RenderUnit.m_IndicesCount; i+=3)
		{
			m_RenderUnit.m_VertexOrder[i + 0] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[0];
			m_RenderUnit.m_VertexOrder[i + 1] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[1];
			m_RenderUnit.m_VertexOrder[i + 2] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[2];
		}

		m_RenderUnit.GenerateRenderBuffer();
	}

	TATRenderUnit m_RenderUnit;

	TATransform m_WorldTransform;

	std::vector<TATCamera*> m_RenderCameras;
	std::vector<TATLight*> m_RenderLights;

	TATMesh* m_RenderMesh;

};