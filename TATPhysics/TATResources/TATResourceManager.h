#pragma once
#include <iostream>
#include "TATPaths.h"
#include "../TATCommon/TATSingleton.h"

using namespace std;

class TATMaterial;
class TATShader;
class TATMesh;
class TATexture;

//TODO Use shared_ptr
class TATResourceManager:public Singleton<TATResourceManager>
{
public:
	TATMaterial* LoadMaterial(const TString& name);

	TATMaterial* LoadDefaultMaterial(const TString& name);

	TATShader* LoadShader(const TString& vs, const TString& fs, const TString& gs);

	TATShader* LoadDefaultShader(const TString& vs, const TString& fs, const TString& gs);

	TATMesh* LoadMesh(const TString& name);

	TATexture* LoadTexture(const TString& name);
};