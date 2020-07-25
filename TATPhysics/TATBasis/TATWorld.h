#pragma once
#include "../TATCommon/TATSingleton.h"
#include "TString.h"
#include <map>

class TATObject;

class TATWorld:public Singleton<TATWorld>
{
public:
	TATWorld();

	std::map<TString, TATObject*> m_GlobalObjects;

	void AddToWorld(const TString& name, TATObject* o);

	void RemoveFromWorld(TATObject* o);
};