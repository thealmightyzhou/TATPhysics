#pragma once
#include "TATManualObject.h"
#include "TATStageNode.h"
#include "../TATApplication/TATWorldListener.h"
#include "../TATResources/TATResourceManager.h"
#include "../TATApplication/TATApplication.h"

class TATLinePainter : public TATManualObject, public TATRenderListener
{
public:
	TATLinePainter()
	{

	}

	void Initialize()
	{
		__super::Initialize();
		TATStageNode* node = TAT_ROOTNODE->CreateChild(TString("linePainter") + TString::ConvertInt(GetObjectIndex()));
		node->MountActor(this);
		node->SetVisible(true);
		m_RenderEleMask.Clear();
		m_RenderMode = TAT_RENDERMODE_LINES;
		m_RenderUnit->m_UseTransform = false;
		m_Material = TATResourceManager::Instance()->LoadDefaultMaterial("lineDrawer.tmaterial");
	}

	void PaintLine(const TATVector3& x, const TATVector3& y, const TATVector3& color)
	{
		m_RenderEleMask.UseColour();

		m_RenderVertices.push_back(TATRenderVertex(x, color));
		m_RenderVertices.push_back(TATRenderVertex(y, color));

		ComputeOrder();
		m_RenderUnit->m_ReadyToRender = true;
		MarkRenderStateDirty();
	}

	void PaintTriangle(const TATVector3& a, const TATVector3& b, const TATVector3& c, const TATVector3& color)
	{
		m_RenderEleMask.UseColour();

		m_RenderVertices.push_back(TATRenderVertex(a, color));
		m_RenderVertices.push_back(TATRenderVertex(b, color));
		m_RenderVertices.push_back(TATRenderVertex(b, color));
		m_RenderVertices.push_back(TATRenderVertex(c, color));
		m_RenderVertices.push_back(TATRenderVertex(c, color));
		m_RenderVertices.push_back(TATRenderVertex(a, color));

		ComputeOrder();
		m_RenderUnit->m_ReadyToRender = true;
		MarkRenderStateDirty();
	}

	void ComputeOrder()
	{
		if(m_RenderOrder.size() != m_RenderVertices.size())
			m_RenderOrder.resize(m_RenderVertices.size());
		for (int i = 0; i < m_RenderVertices.size(); ++i)
		{
			m_RenderOrder[i] = i;
		}
	}

	virtual void RenderOneFrameEnd(float dt) override
	{

	}

	void Clear()
	{
		m_RenderVertices.clear();
		m_RenderOrder.clear();
	}
};