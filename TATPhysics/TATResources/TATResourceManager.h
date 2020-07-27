#pragma once
#include <iostream>
#include "TATPaths.h"
#include "../TATCommon/TATSingleton.h"
#include "TATMesh.h"
#include "TATexture.h"
#include "TATShader.h"
#include "TATMaterial.h"

using namespace std;

class TATResourceManager:public Singleton<TATResourceManager>
{
public:
	shared_ptr<TATMaterial> LoadMaterial(const TString& name)
	{
		TATMaterial* material = new TATMaterial(name);
		material->Load(name);
		return shared_ptr<TATMaterial>(material);
	}

	shared_ptr<TATMesh> LoadMesh(const TString& name)
	{
		TATMesh* mesh = new TATMesh(name);
		mesh->Load(name);
		return shared_ptr<TATMesh>(mesh);
	}

	shared_ptr<TATexture> LoadTexture(const TString& name)
	{
		TATexture* tex = new TATexture;
		tex->Load(name);
		return shared_ptr<TATexture>(tex);
	}
};