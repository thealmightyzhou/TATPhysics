#include "TATObject.h"
#include "../TATApplication/TATApplication.h"
#include "TATWorld.h"

int TATObject::m_ObjectIndex = 0;

TATObject::TATObject(const TString& name):m_GlobalName(name)
{
	
}

TATObject::~TATObject()
{
	TATWorld::Instance()->RemoveFromWorld(this);
}

//called after construction immediately;
void TATObject::Initialize()
{
	TATWorld::Instance()->AddToWorld(m_GlobalName, this);
}