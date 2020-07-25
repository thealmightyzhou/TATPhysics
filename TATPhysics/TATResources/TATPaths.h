#pragma once

#include "../TATBasis/TString.h"
#include <Windows.h>

class TATPaths
{
public:
	static TString PathOfProj()
	{
		return PathOfExe().UpperDirect();
	}

	static TString PathOfExe()
	{
		char chpath[MAX_PATH];
		GetModuleFileName(NULL, (LPSTR)chpath, sizeof(chpath));
		return TString(chpath);
	}

	static TString PathOfResourceRoot(const TString& appName)
	{
		return PathOfProj().Visit(appName);
	}

	static TString PathOfShaderRoot(const TString& appName)
	{
		return PathOfResourceRoot(appName).Visit("Shaders");
	}

	static TString PathOfTextureRoot(const TString& appName)
	{
		return PathOfResourceRoot(appName).Visit("Textures");
	}

	static TString PathOfModelRoot(const TString& appName)
	{
		return PathOfResourceRoot(appName).Visit("Models");
	}

	static TString PathOfShader(const TString& appName, const TString& name)
	{
		return PathOfShaderRoot(appName).Visit(name);
	}

	static TString PathOfModel(const TString& appName, const TString& name)
	{
		return PathOfModelRoot(appName).Visit(name);
	}

	static TString PathOfTexture(const TString& appName, const TString& name)
	{
		return PathOfTextureRoot(appName).Visit(name);
	}
};