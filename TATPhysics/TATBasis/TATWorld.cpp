#include "TATWorld.h"
#include "TATObject.h"
#include "../TATResources/TATexture.h"
#include "../TATResources/TATMaterial.h"
#include "../TATResources/TATMesh.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"

void TATWorld::AddToWorld(const TString& name, TATObject* o)
{
	m_GlobalObjects.insert(std::make_pair(name, o));
}

void TATWorld::RemoveFromWorld(TATObject* o)
{
	if (o && m_GlobalObjects[o->GetGlobalName()])
	{
		delete m_GlobalObjects[o->GetGlobalName()];
		m_GlobalObjects.erase(o->GetGlobalName());
	}
}

TATObject* TATWorld::GetObjectByName(const TString& name)
{
	return m_GlobalObjects[name];
}

TATCamera* TATWorld::GetCamera(const TString& name)
{
	return GetObjectByName("camera_" + name)->Cast<TATCamera>();
}

TATMesh* TATWorld::GetMesh(const TString& name)
{
	return GetObjectByName("mesh_" + name)->Cast<TATMesh>();
}

TATMaterial* TATWorld::GetMaterial(const TString& name)
{
	return GetObjectByName("material_" + name)->Cast<TATMaterial>();
}

TATLight* TATWorld::GetLight(const TString& name)
{
	return GetObjectByName("light_" + name)->Cast<TATLight>();
}

TATexture* TATWorld::GetTexture(const TString& name)
{
	return GetObjectByName("texture_" + name)->Cast<TATexture>();
}