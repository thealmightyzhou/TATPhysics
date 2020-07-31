#include "TATResourceManager.h"
#include "TATMesh.h"
#include "TATexture.h"
#include "TATShader.h"
#include "TATMaterial.h"
#include "../TATBasis/TATErrorReporter.h"

TATMaterial* TATResourceManager::LoadDefaultMaterial(const TString& name)
{
	TATMaterial* material = new TATMaterial(name.FetchInnerName());
	TString path = TATPaths::PathOfDefaultMaterial(name);
	if (!TATFileStream::FileExists(path))
	{
		TATErrorReporter::Instance()->ReportFileMissed("default material_" + name);
		return 0;
	}

	material->Load(path);

	TString vPath, fPath, gPath;
	material->GetShaderPaths(vPath, fPath, gPath);
	material->m_Shader = TATResourceManager::Instance()->LoadDefaultShader(vPath, fPath, gPath);

	return material;
}

TATMaterial* TATResourceManager::LoadMaterial(const TString& name)
{
	TATMaterial* material = new TATMaterial(name.FetchInnerName());
	TString path = TATPaths::PathOfMaterial(TAT_APPNAME, name);
	if (!TATFileStream::FileExists(path))
	{
		TATErrorReporter::Instance()->ReportFileMissed("material_" + name);
		return 0;
	}

	material->Load(path);

	TString vPath, fPath, gPath;
	material->GetShaderPaths(vPath, fPath, gPath);
	material->m_Shader = TATResourceManager::Instance()->LoadShader(vPath, fPath, gPath);

	return material;
}

TATShader* TATResourceManager::LoadShader(const TString& vs, const TString& fs, const TString& gs)
{
	TString vPath = TATPaths::PathOfShader(TAT_APPNAME, vs);
	TString fPath = TATPaths::PathOfShader(TAT_APPNAME, fs);
	TString gPath = TATPaths::PathOfShader(TAT_APPNAME, gs);
	if (!TATFileStream::FileExists(vPath) || !TATFileStream::FileExists(fPath))
	{
		TATErrorReporter::Instance()->ReportFileMissed("shader_" + vPath + " or shader_" + fPath);
		return 0;
	}
	TATShader* shader = new TATShader(vPath, fPath, gPath);
	return shader;
}

TATShader* TATResourceManager::LoadDefaultShader(const TString& vs, const TString& fs, const TString& gs)
{
	TString vPath = TATPaths::PathOfDefaultShader(vs);
	TString fPath = TATPaths::PathOfDefaultShader(fs);
	TString gPath = TATPaths::PathOfDefaultShader(gs);
	if (!TATFileStream::FileExists(vPath) || !TATFileStream::FileExists(fPath))
	{
		TATErrorReporter::Instance()->ReportFileMissed("default shader_" + vPath + " or default shader_" + fPath);
		return 0;
	}
	TATShader* shader = new TATShader(vPath, fPath, gPath);
	return shader;
}

TATMesh* TATResourceManager::LoadMesh(const TString& name)
{
	TATMesh* mesh = new TATMesh(name.FetchInnerName());
	TString filePath = TATPaths::PathOfModel(TAT_APPNAME, name);
	if (!TATFileStream::FileExists(filePath))
	{
		TATErrorReporter::Instance()->ReportFileMissed("mesh_" + name);
		return 0;
	}
	mesh->Load(filePath);
	return mesh;
}

TATexture* TATResourceManager::LoadTexture(const TString& name)
{
	TATexture* tex = new TATexture(name.FetchInnerName());
	TString filePath = TATPaths::PathOfTexture(TAT_APPNAME, name);
	if (!TATFileStream::FileExists(filePath))
	{
		TATErrorReporter::Instance()->ReportFileMissed("texture_" + name);
		return 0;
	}
	tex->Load(filePath);
	return tex;
}