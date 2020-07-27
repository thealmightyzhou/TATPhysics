#include "TATWorld.h"
#include "TATObject.h"
#include "../TATResources/TATexture.h"
#include "../TATResources/TATMaterial.h"
#include "../TATResources/TATMesh.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"
#include "../TATStage/TATStageNode.h"

void TATWorld::AddToWorld(const TString& name, TATObject* o)
{
	m_GlobalObjects.insert(std::make_pair(name, o));
}

bool TATWorld::AddToStage(const TString& name, TATStageNode* a)
{
	if (m_StageNodes.find(name) != m_StageNodes.end())
		return false;
	
	m_StageNodes.insert(std::make_pair(name, a));
	return true;
}

void TATWorld::RemoveFromWorld(TATObject* o)
{
	if (o && m_GlobalObjects[o->GetGlobalName()])
	{
		delete m_GlobalObjects[o->GetGlobalName()];
		m_GlobalObjects.erase(o->GetGlobalName());
	}
}

bool TATWorld::RemoveFromStage(TATStageNode* a)
{
	if (a && m_StageNodes[a->GetName()])
	{
		m_StageNodes.erase(a->GetName());
		a->DestroySelf();

		return true;
	}
	
	return false;
}

TATObject* TATWorld::GetObjectByName(const TString& name)
{
	return m_GlobalObjects[name];
}

TATStageNode* TATWorld::GetNodeByName(const TString& name)
{
	return m_StageNodes[name];
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