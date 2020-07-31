#pragma once
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATStage/TATActor.h"
#include "../TATResources/TATObjLoader.h"
#include "../TATGLRender/TATGLRenderer.h"

class TATManualObject : public TATActor
{
public:
	TATManualObject();

	virtual ~TATManualObject()
	{

	}

	virtual void FillRenderUnit() override;

	void SetRenderMode(int mode)
	{
		m_RenderMode = mode;
	}

	std::vector<TATRenderVertex> m_RenderVertices;
	std::vector<int> m_RenderOrder;

	TATModelElementMask m_RenderEleMask;
	bool m_ReadyToRender;
	int m_RenderMode;
};