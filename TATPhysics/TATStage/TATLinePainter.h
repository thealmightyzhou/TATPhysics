#pragma once
#include "TATManualObject.h"
#include "TATStageNode.h"
#include "../TATApplication/TATWorldListener.h"
#include "../TATResources/TATResourceManager.h"

class TATLinePainter : public TATManualObject,public TATRenderListener
{
public:
	TATLinePainter()
	{
		TATStageNode* node = TAT_ROOTNODE->CreateChild("linePainter" + GetObjectIndex());
		node->MountActor(this);
		node->SetVisible(true);
		m_RenderMode = TAT_RENDERMODE_LINES;
		m_RenderEleMask.Clear();
		m_RenderUnit->m_UseTransform = false;
		m_Material = TATResourceManager::Instance()->LoadDefaultMaterial("lineDrawer.tmaterial");
	}

	void PaintLine(const TATVector3& x, const TATVector3& y,const TATVector3& color)
	{
		m_RenderEleMask.UseColour();

		m_RenderVertices.push_back(TATRenderVertex(x, color));
		m_RenderVertices.push_back(TATRenderVertex(y, color));

		ComputeOrder();
		m_ReadyToRender = true;
	}

	void ComputeOrder()
	{
		m_RenderOrder.resize(m_RenderVertices.size());
		for (int i = 0; i < m_RenderVertices.size(); ++i)
		{
			m_RenderOrder[i] = i;
		}
	}

	virtual void RenderOneFrameEnd(float dt) override
	{
		m_RenderVertices.clear();
		m_RenderOrder.clear();
	}
};