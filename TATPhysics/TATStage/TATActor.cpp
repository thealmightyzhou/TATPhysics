#include "TATActor.h"
#include "../TATApplication/TATApplication.h"
#include "../TATApplication/TAThread.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"
#include "../TATBasis/TATWorld.h"
#include "../TATResources/TATMesh.h"
#include "../TATResources/TATMaterial.h"
#include "../TATDynamics/TATDynamicWorld.h"
#include "../TATBasis/TATickable.h"

TATActor::TATActor() : TATObject("actor_manual_" + TString::Make(GetObjectIndex()))
{
	m_RenderMesh = 0;
}

TATActor::TATActor(TATMesh* ptr) :TATObject("actor_" + ptr->GetSubName() + TString::Make(GetObjectIndex()))
{
	m_RenderMesh = ptr;
}

TATActor::~TATActor()
{
	m_RenderMesh = 0;
	m_RenderCamera = 0;
	TAT_RENDER_THREAD->m_RenderUnitPool.ReturnUsed(m_RenderUnit);
	m_RenderUnit = 0;
}

void TATActor::Initialize()
{
	__super::Initialize();
	m_RenderCamera = TATWorld::Instance()->GetCamera("main");
	m_RenderUnit = TAT_RENDER_THREAD->m_RenderUnitPool.FetchUnused();
	m_WorldTransform = TATransform::GetIdentity();
	m_RenderStateDirty = true;
}

void TATActor::SetUseTransform(bool flag)
{
	m_RenderUnit->m_UseTransform = flag;
}

void TATActor::FillRenderUnit()
{
	if (!m_RenderMesh || !m_RenderStateDirty)
		return;

	m_RenderUnit->m_RenderEleMask = m_RenderMesh->m_ModelElementMask;

	m_RenderCamera->GetViewMatrix(m_RenderUnit->m_MatrixView);
	m_RenderCamera->GetProjectionMatrix(m_RenderUnit->m_MatrixProj);
	if (m_RenderUnit->m_UseTransform)
		m_WorldTransform.GetOpenGLMatrix(m_RenderUnit->m_MatrixModel);
	else
		TATransform::GetIdentity().GetOpenGLMatrix(m_RenderUnit->m_MatrixModel);

	m_RenderUnit->m_VertexCount = m_RenderMesh->m_VertexCount;

	TAT_SAFE_NEW(m_RenderUnit->m_RenderVertices, TATRenderVertex, m_RenderUnit->m_VertexCount);

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

		if (m_RenderUnit->m_RenderEleMask.IsUseColour())
		{
			vertices[i].m_Colour = m_RenderMesh->m_MeshVertices[i].m_Colour;
		}

		if (m_RenderUnit->m_RenderEleMask.IsUseTexCoordinate())
		{
			vertices[i].m_TexCoordinateCount = m_RenderUnit->m_RenderEleMask.m_TexCount;
			for (int u = 0; u < vertices[i].m_TexCoordinateCount; u++)
			{
				vertices[i].m_TexCoordinates[u][0] = m_RenderMesh->m_MeshVertices[i].m_TexCoordinate[u][0];
				vertices[i].m_TexCoordinates[u][1] = m_RenderMesh->m_MeshVertices[i].m_TexCoordinate[u][1];
			}
		}
	}

	m_RenderUnit->m_IndicesCount = 3 * m_RenderMesh->m_FaceCount;

	TAT_SAFE_NEW(m_RenderUnit->m_VertexOrder, int, m_RenderUnit->m_IndicesCount);

	for (int i = 0; i < m_RenderUnit->m_IndicesCount; i += 3)
	{
		m_RenderUnit->m_VertexOrder[i + 0] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[0];
		m_RenderUnit->m_VertexOrder[i + 1] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[1];
		m_RenderUnit->m_VertexOrder[i + 2] = m_RenderMesh->m_MeshFaces[i / 3].m_Vertices[2];
	}

	m_RenderUnit->m_TexCoordinateCount = m_RenderMesh->m_Loader->m_ModelElementMask.m_TexCount;
	m_RenderUnit->m_TexCount = m_RenderUnit->m_TexCoordinateCount;
	TAT_MEMCPY(m_RenderUnit->m_Textures, m_Material->m_Textures);

	m_RenderUnit->GenerateRenderBuffer();

	m_RenderUnit->m_ReadyToRender = true;

	m_RenderStateDirty = false;
}

void TATActor::SetMaterial(TATMaterial* m)
{
	m_Material = m;
	m_RenderUnit->SetMaterial(m);
	m_RenderCamera = m->m_Camera;
}

bool TATActor::Update(float dt)
{
	TATransform tr = m_RelativeTransform * m_WorldTransform;

	if (m_RenderUnit)
	{
		m_RenderUnit->m_Transform = tr;
	}

	for (int i = 0; i < m_TickableObjects.size(); ++i)
	{
		m_TickableObjects[i]->Update(this, dt);
	}

	return true;
}

void TATActor::MarkRenderStateDirty()
{
	m_RenderStateDirty = true;
	if (m_RenderUnit)
		m_RenderUnit->m_StaticDataUploaded = false;

	TAT_RENDER_THREAD->MarkRenderStateDirty();
}

void TATActor::AttachTickable(TATickable* x)
{
	m_TickableObjects.push_back(x);
	x->m_HostActor = this;
}