#pragma once
#include <iostream>
#include "TATPaths.h"
#include "../TATCommon/TATSingleton.h"
#include "TATMesh.h"
#include "TATexture.h"
#include "TATShader.h"
#include "TATMaterial.h"

using namespace std;

//TODO Use shared_ptr
class TATResourceManager:public Singleton<TATResourceManager>
{
public:
	TATMaterial* LoadMaterial(const TString& name)
	{
		TATMaterial* material = new TATMaterial(name.FetchInnerName());
		material->Load(name);
		return material;
	}

	TATMesh* LoadMesh(const TString& name)
	{
		TATMesh* mesh = new TATMesh(name.FetchInnerName());
		mesh->Load(name);
		return mesh;
	}

	TATexture* LoadTexture(const TString& name)
	{
		TATexture* tex = new TATexture(name.FetchInnerName());
		tex->Load(name);
		return tex;
	}
};