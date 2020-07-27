#pragma once

#include "../TATBasis/TATObject.h"
#include "../TATCommon/TATransform.h"

class TATRenderUnit;
class TATCamera;
class TATLight;
class TATMesh;

//base object to show on stage
class TATActor :public TATObject
{
public:
	TATActor(TATMesh* ptr);

	virtual ~TATActor();

	void FillRenderUnit();

	TATRenderUnit* m_RenderUnit;

	TATransform m_WorldTransform;

	TATCamera* m_RenderCamera;

	TATLight* m_RenderLight;

	TATMesh* m_RenderMesh;

	int m_RigidBodyId;

};