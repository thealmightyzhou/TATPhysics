#include "TATMaterial.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATBasis/TATObject.h"
#include "../TATBasis/TATWorld.h"
#include "TATShader.h"
#include "TATPaths.h"
#include "TATFileStream.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"
#include "TATResourceManager.h"
#include "../TATResources/TATexture.h"

void TATMaterial::Load(const TString& path)
{
	__super::Load(path);

	std::vector<TString> strs;
	TATFileStream::ReadFileToLines(path, strs);

	for (int i = 0; i < (int)strs.size(); i++)
	{
		//TODO 
		//if has '$' 
		//TATLightFactory factory;
		//factory.GenerateLight()

		std::vector<TString> pair;
		strs[i].Split(":", pair);

		if (m_MaterialSetting.find(pair[0]) != m_MaterialSetting.end())
		{
			m_MaterialSetting[pair[0]] = pair[1];
		}

		
	}

	

	if (m_MaterialSetting["CameraName"] != "")
	{
		m_Camera = TATWorld::Instance()->GetCamera(m_MaterialSetting["CameraName"]);
	}

	for (int i = 0; i < 5; i++)
	{
		if (m_MaterialSetting[TString("TextureUnit") + TString::ConvertInt(i)] != "")
		{
			m_Textures[i] = TATResourceManager::Instance()->LoadTexture(m_MaterialSetting[TString("TextureUnit") + TString::ConvertInt(i)]);
		}
	}
}

void TATMaterial::OnMaterialSetted(TATRenderUnit* unit)
{
	m_RenderUnits.push_back(unit);
}