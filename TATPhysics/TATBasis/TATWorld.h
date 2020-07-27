#pragma once
#include "../TATCommon/TATSingleton.h"
#include "TString.h"
#include <map>

class TATObject;
class TATCamera;
class TATMaterial;
class TATexture;
class TATLight;
class TATMesh;
class TATStageNode;

class TATWorld:public Singleton<TATWorld>
{
public:
	TATWorld() {}

	std::map<TString, TATObject*> m_GlobalObjects;

	std::map<TString, TATStageNode*> m_StageNodes;

	void AddToWorld(const TString& name, TATObject* o);

	bool AddToStage(const TString& name, TATStageNode* a);

	void RemoveFromWorld(TATObject* o);

	bool RemoveFromStage(TATStageNode* a);

	TATObject* GetObjectByName(const TString& name);

	TATStageNode* GetNodeByName(const TString& name);

	TATCamera* GetCamera(const TString& name);

	TATMesh* GetMesh(const TString& name);

	TATMaterial* GetMaterial(const TString& name);

	TATLight* GetLight(const TString& name);

	TATexture* GetTexture(const TString& name);
};