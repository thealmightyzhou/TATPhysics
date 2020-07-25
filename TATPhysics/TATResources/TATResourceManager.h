#pragma once
#include <iostream>
#include "../TATBasis/TString.h"
#include "TATPaths.h"
#include "../TATCommon/TATSingleton.h"
#include "TATMesh.h"
#include "TATexture.h"
#include "TATShader.h"

using namespace std;

class TATResourceManager:public Singleton<TATResourceManager>
{
public:
	shared_ptr<TATMesh> LoadMesh(const TString& name)
	{
		TATMesh* mesh = new TATMesh;
		mesh->Load(name);
		return shared_ptr<TATMesh>(mesh);
	}

	shared_ptr<TATexture> LoadTexture(const TString& name)
	{
		TATexture* tex = new TATexture;
		tex->Load(name);
		return shared_ptr<TATexture>(tex);
	}

	shared_ptr<TATShader> LoadShader(const TString& vsName, const TString& fsName)
	{
		TString vsPath = TATPaths::PathOfShader(TATApplication::Instance()->GetAppName(), vsName);
		TString fsPath = TATPaths::PathOfShader(TATApplication::Instance()->GetAppName(), fsName);

		TATShader* shader = new TATShader(vsPath.ToChar(), fsPath.ToChar());

		return shared_ptr<TATShader>(shader);
	}
};