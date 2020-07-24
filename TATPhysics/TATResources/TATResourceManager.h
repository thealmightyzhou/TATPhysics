#pragma once
#include <iostream>
#include "../TATBasis/TString.h"
#include "TATResourcePrimitive.h"
#include "TATPaths.h"
//TODO singleton

using namespace std;

class TATResourceManager
{
public:
	static shared_ptr<TATMesh> LoadMesh(const TString& path)
	{
		TATMesh* mesh = new TATMesh(path);
		return shared_ptr<TATMesh>(mesh);
	}

};