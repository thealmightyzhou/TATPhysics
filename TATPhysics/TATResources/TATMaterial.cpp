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
		std::vector<TString> pair;
		strs[i].Split(":", pair);

		if (m_MaterialSetting.find(pair[0]) != m_MaterialSetting.end())
		{
			m_MaterialSetting[pair[0]] = pair[1];
		}
	}

	if (m_MaterialSetting["LightName"] != "")
	{
		m_Light = TATWorld::Instance()->GetLight(m_MaterialSetting["LightName"]);
	}

	if (m_Light)
	{
		if (m_Light->GetType() == LIGHTTYPE::DIRECTIONAL_LIGHT)
			m_LightDirection = m_Light->Cast<TATDirectionLight>()->GetDirection();
		else if (m_Light->GetType() == LIGHTTYPE::POINT_LIGHT)
		{
			TATPointLight* pl = m_Light->Cast<TATPointLight>();
			m_PointLightConstant = pl->GetConstant();
			m_PointLightLinear = pl->GetLinear();
			m_PointLightExp = pl->GetExp();
		}

		m_LightColor = m_Light->GetColor();
		m_LightAmbient = m_Light->GetAmbient();
		m_LightDiffuse = m_Light->GetDiffuse();
		m_LightSpecularIntensity = m_Light->GetSpecularIntensity();
		m_LightSpecularPower = m_Light->GetSpecularPower();
		m_LightType = m_Light->GetType();
		m_LightPosition = m_Light->GetPosition();

	}

	if (m_MaterialSetting["LightColor"] != "")
		m_LightColor = m_MaterialSetting["LightColor"].ToVector3();
	if (m_MaterialSetting["LightAmbient"] != "")
		m_LightAmbient = m_MaterialSetting["LightAmbient"].ToFloat();
	if (m_MaterialSetting["LightDiffuse"] != "")
		m_LightDiffuse = m_MaterialSetting["LightDiffuse"].ToFloat();
	if (m_MaterialSetting["LightSpecularIntensity"] != "")
		m_LightSpecularIntensity = m_MaterialSetting["LightSpecularIntensity"].ToFloat();
	if (m_MaterialSetting["LightSpecularPower"] != "")
		m_LightSpecularPower = m_MaterialSetting["LightSpecularPower"].ToFloat();
	if (m_MaterialSetting["PointLightConstant"] != "")
		m_PointLightConstant = m_MaterialSetting["PointLightConstant"].ToFloat();
	if (m_MaterialSetting["PointLightLinear"] != "")
		m_PointLightLinear = m_MaterialSetting["PointLightLinear"].ToFloat();
	if (m_MaterialSetting["PointLightExp"] != "")
		m_PointLightExp = m_MaterialSetting["PointLightExp"].ToFloat();

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