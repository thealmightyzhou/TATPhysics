#pragma once
#include "TATObject.h"

class TATActor;

class TATickable:public TATObject
{
public:
	TATickable(const TString& name) :TATObject(name)
	{}

	virtual ~TATickable() {}

	virtual bool Update(TATActor* actor, float dt);

	TATActor* m_HostActor;
};