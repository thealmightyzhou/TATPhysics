#include "TATObject.h"
#include "../TATApplication/TATApplication.h"
#include "TATWorld.h"

int TATObject::m_ObjectIndex = 0;

TATObject::TATObject(const TString& name):m_GlobalName(name)
{
	TATWorld::Instance()->AddToWorld(name ,this);
}

TATObject::~TATObject()
{
	TATWorld::Instance()->RemoveFromWorld(this);
}