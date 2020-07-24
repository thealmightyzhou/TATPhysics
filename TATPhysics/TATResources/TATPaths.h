#pragma once

#include "../TATBasis/TString.h"

class TATPaths
{
public:
	static TString PathOfProj()
	{

	}

	static TString PathOfExe()
	{

	}

	static TString PathOfResourceRoot(const TString& appName)
	{

	}

	static TString PathOfShaderRoot(const TString& appName)
	{

	}

	static TString PathOfTextureRoot(const TString& appName)
	{

	}

	static TString PathOfModelRoot(const TString& appName)
	{

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