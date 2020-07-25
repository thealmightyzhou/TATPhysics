#include "TATWorld.h"
#include "TATObject.h"

void TATWorld::AddToWorld(const TString& name, TATObject* o)
{
	m_GlobalObjects.insert(std::make_pair(name, o));
}

void TATWorld::RemoveFromWorld(TATObject* o)
{
	if (o && m_GlobalObjects[o->GetGlobalName()])
	{
		delete m_GlobalObjects[o->GetGlobalName()];
		m_GlobalObjects.erase(o->GetGlobalName());
	}
}