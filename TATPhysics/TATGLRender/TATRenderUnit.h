#pragma once
#include "TATGLHeader.h"
#include "../TATCommon/TATCore.h"
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATransform.h"

enum TATRenderPrimitiveMask
{
	TATRenderVertexMask,
	TATRenderNormalMask,
	TATRenderTexCoordinateMask,
	TATRenderTextureMask,
	TATRenderTangentMask
};

struct TATRenderVertex
{
public:
	TATVector3 m_Position;
	TATVector3 m_Normal;
	TATVector3 m_Tangent;
	float** m_TexCoordinates;
	int m_TexCoordinateCount;
};

class TATRenderUnit
{
public:
	TATRenderUnit()
	{
		m_RenderUintMask = 0;
	}

	int m_VertexCount;
	int m_NormalCount;
	int m_TexCoordinateCount;
	int m_TangentCount;
	int m_RenderVertexCount;

	float* m_RenderBuffer; //a basic unit to render include vertex,normal,uv,tangent which transport to glbuffer

	TATRenderVertex* m_RenderVertices; //already sort by face's vertex indices

	UINT m_RenderUintMask;
	UINT* m_TextureIds;
	UINT m_ShaderId;

	TATransform m_Transform;

	bool m_UseTransform; //transform to wcs or already in wcs

	inline void UseNormal()
	{
		m_RenderUintMask |= TATRenderNormalMask;
	}

	inline void UseTexCoordinate()
	{
		m_RenderUintMask |= TATRenderTexCoordinateMask;
	}

	inline void UseTexture()
	{
		m_RenderUintMask |= TATRenderTextureMask;
	}

	inline void UseTangent()
	{
		m_RenderUintMask |= TATRenderTangentMask;
	}

	inline bool IsUseNormal()
	{
		return (m_RenderUintMask & TATRenderNormalMask);
	}

	inline bool IsUseTexCoordinate()
	{
		return (m_RenderUintMask & TATRenderTexCoordinateMask);
	}

	inline bool IsUseTexture()
	{
		return (m_RenderUintMask & TATRenderTextureMask);
	}

	inline bool IsUseTangent()
	{
		return (m_RenderUintMask & TATRenderTangentMask);
	}
};