#pragma once

#include "../TATBasis/TATObject.h"
#include "../TATCommon/TATransform.h"

class TATRenderUnit;
class TATCamera;
class TATLight;
class TATMesh;
class TATMaterial;
class TATickable;

//base object to show on stage
class TATActor :public TATObject
{
public:
	TATActor(TATMesh* ptr);

	TATActor();

	virtual ~TATActor();

	void Initialize();

	virtual void FillRenderUnit();

	void SetMaterial(TATMaterial* m);

	virtual bool Update(float dt);

	//notify the actor to reupload vertex datas
	void MarkRenderStateDirty();

	void AttachTickable(TATickable* x);

	void SetUseTransform(bool flag);

	void SetRelativeTransform(const TATransform& tr)
	{
		m_RelativeTransform = tr;
	}

	TATRenderUnit* m_RenderUnit;

	TATransform m_WorldTransform;

	TATransform m_RelativeTransform;

	TATCamera* m_RenderCamera;

	TATMesh* m_RenderMesh;

	TATMaterial* m_Material;

	std::vector<TATickable*> m_TickableObjects;

	bool m_RenderStateDirty;

};