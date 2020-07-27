#pragma once
#include "TATGLHeader.h"
#include "../TATCommon/TATransform.h"
#include "../TATResources/TATModelLoader.h"
#include "../TATResources/TATexture.h"

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
	TATRenderUnit();

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

	bool m_ReadyToRender; //if true render this unit

	TATRenderVertex* m_RenderVertices; //already sort by face's vertex indices

	TATexture* m_Textures[5];
	UINT* m_TextureIds;
	UINT m_ShaderId;
	UINT m_VAOId;
	UINT m_VBOId;

	TATransform *m_TransformPtr;

	bool m_UseTransform; //transform to wcs or already in wcs

	TATModelElementMask m_RenderEleMask;

	bool m_StaticDataUploaded;

	void SetMaterial(TATMaterial* mat);

	void GenerateRenderBuffer();
};