#pragma once

#include "../TATBasis/TATObject.h"
#include "../TATCommon/TATransform.h"

class TATRenderUnit;
class TATCamera;
class TATLight;
class TATMesh;
class TATMaterial;

//base object to show on stage also can relative with rigidbody by m_RigidBodyId
class TATActor :public TATObject
{
public:
	TATActor(TATMesh* ptr);

	TATActor();

	virtual ~TATActor();

	virtual void FillRenderUnit();

	void SetMaterial(TATMaterial* m);

	virtual void Update(float dt);

	void SetRigidBody(int index);

	TATRenderUnit* m_RenderUnit;

	TATransform m_WorldTransform;

	TATCamera* m_RenderCamera;

	TATLight* m_RenderLight;

	TATMesh* m_RenderMesh;

	TATMaterial* m_Material;

	int m_RigidBodyId;

};