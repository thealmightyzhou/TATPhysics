#pragma once
#include <iostream>
#include "../TATBasis/TString.h"
#include "../TATBasis/TATObject.h"

using namespace std;

class TATResourcePrimitive:public TATObject
{
public:
	TATResourcePrimitive(const TString& name):TATObject(name)
	{}

	virtual ~TATResourcePrimitive()
	{

	}

	virtual void Load(const TString& path)
	{
		m_Path = path;
	}

	TString m_Path;
};
