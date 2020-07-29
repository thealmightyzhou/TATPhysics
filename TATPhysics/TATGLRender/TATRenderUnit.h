#pragma once
#include "TATGLHeader.h"
#include "../TATCommon/TATransform.h"
#include "../TATResources/TATModelLoader.h"
#include "../TATResources/TATexture.h"
#include "../TATCommon/TATCore.h"

class TATMaterial;

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
	TAT_POOL_OBJECT(TATRenderUnit);

	TATMaterial* m_Material;

	int m_VertexCount;
	int m_TriangleCount;
	int m_TexCoordinateCount;
	int m_IndicesCount;
	int m_TexCount;
	int m_BlockSize;

	float* m_RenderBuffer; //a basic unit to render include vertex,normal,uv,tangent which transport to glbuffer
	int* m_VertexOrder; //indices can be divided by 3;

	float m_MatrixView[16];
	float m_MatrixProj[16];
	float m_MatrixModel[16];

	bool m_ReadyToRender; //if true render this unit
	bool m_StaticDataUploaded;
	bool m_UseTransform; //transform to wcs or already in wcs

	TATRenderVertex* m_RenderVertices; //already sort by face's vertex indices

	TATransform m_Transform;

	TATexture* m_Textures[5];
	UINT m_TextureIds[5];
	UINT m_ShaderId;
	UINT m_VAOId;
	UINT m_VBOId;

	TATModelElementMask m_RenderEleMask;

	void SetMaterial(TATMaterial* mat);

	void GenerateRenderBuffer();
};