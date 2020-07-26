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

class TATWorld:public Singleton<TATWorld>
{
public:
	TATWorld();

	std::map<TString, TATObject*> m_GlobalObjects;

	void AddToWorld(const TString& name, TATObject* o);

	void RemoveFromWorld(TATObject* o);

	TATObject* GetObjectByName(const TString& name);

	TATCamera* GetCamera(const TString& name);

	TATMesh* GetMesh(const TString& name);

	TATMaterial* GetMaterial(const TString& name);

	TATLight* GetLight(const TString& name);

	TATexture* GetTexture(const TString& name);
};