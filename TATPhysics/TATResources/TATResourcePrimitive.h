#pragma once
#include <iostream>
#include "../TATBasis/TString.h"

using namespace std;

class TATResourcePrimitive
{
public:
	TATResourcePrimitive() {}

	TATResourcePrimitive(const TString& path) :m_Path(path)
	{
		Load(path);
	}

	virtual ~TATResourcePrimitive()
	{

	}

	virtual void Load(const TString& path)
	{
		m_Path = path;
	}

	TString m_Path;
};

class TATMesh:public TATResourcePrimitive
{
public:
	TATMesh(const TString& path) :TATResourcePrimitive(path)
	{}
};