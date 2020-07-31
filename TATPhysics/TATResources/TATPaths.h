#pragma once

#include "../TATBasis/TString.h"
#include <Windows.h>

class TATPaths
{
public:
	static TString PathOfProj()
	{
		TString res = PathOfExe().UpperDirect().UpperDirect().UpperDirect();
		return res;
	}

	static TString PathOfExe()
	{
		TCHAR chpath[MAX_PATH];
		GetModuleFileName(NULL, chpath, MAX_PATH);
		char path[MAX_PATH];
		for (int i = 0; i < MAX_PATH; i++)
		{
			path[i] = chpath[i];
		}

		std::vector<TString> strs;
		TString(path).Split("\\", strs);

		for (int i = 1; i < strs.size(); i++)
		{
			strs[0] = strs[0].Visit(strs[i]);
		}

		return strs[0];
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

	static TString PathOfMaterialRoot(const TString& appName)
	{
		return PathOfResourceRoot(appName).Visit("Materials");
	}

	static TString PathOfShader(const TString& appName, const TString& name)
	{
		if (name.IsEmpty())
			return "";
		return PathOfShaderRoot(appName).Visit(name);
	}

	static TString PathOfModel(const TString& appName, const TString& name)
	{
		if (name.IsEmpty())
			return "";
		return PathOfModelRoot(appName).Visit(name);
	}

	static TString PathOfTexture(const TString& appName, const TString& name)
	{
		if (name.IsEmpty())
			return "";
		return PathOfTextureRoot(appName).Visit(name);
	}

	static TString PathOfMaterial(const TString& appName, const TString& name)
	{
		if (name.IsEmpty())
			return "";
		return PathOfMaterialRoot(appName).Visit(name);
	}

	static TString PathOfDefaultMaterial(const TString& name)
	{
		return PathOfProj().Visit("Default").Visit("Materials").Visit(name);
	}

	static TString PathOfDefaultShader(const TString& name)
	{
		return PathOfProj().Visit("Default").Visit("Shaders").Visit(name);
	}
};