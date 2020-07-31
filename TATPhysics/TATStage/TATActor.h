#pragma once

#include "../TATBasis/TATObject.h"
#include "../TATCommon/TATransform.h"

class TATRenderUnit;
class TATCamera;
class TATLight;
class TATMesh;
class TATMaterial;

//base object to show on stage
class TATActor :public TATObject
{
public:
	TATActor(TATMesh* ptr);

	TATActor();

	virtual ~TATActor();

	virtual void FillRenderUnit();

	void SetMaterial(TATMaterial* m);

	TATRenderUnit* m_RenderUnit;

	TATransform m_WorldTransform;

	TATCamera* m_RenderCamera;

	TATLight* m_RenderLight;

	TATMesh* m_RenderMesh;

	TATMaterial* m_Material;

	int m_RigidBodyId;

};