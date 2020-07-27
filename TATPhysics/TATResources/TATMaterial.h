#pragma once
#include "../TATBasis/TString.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "TATFileStream.h"
#include <iostream>
#include "../TATBasis/TATObject.h"
#include "../TATBasis/TATWorld.h"
#include "TATShader.h"
#include "TATPaths.h"
#include "TATFileStream.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"


using namespace std;

class TATMaterial :public TATResourcePrimitive
{
public:
	TATMaterial(const TString& name) : TATResourcePrimitive("material_" + name)
	{
		m_Light = 0;
		m_Shader = 0;

		Register();
	}

	void Register()
	{
		m_MaterialSetting["VertexShader"] = "";
		m_MaterialSetting["FragmentShader"] = "";
		m_MaterialSetting["LightName"] = "main";
		m_MaterialSetting["LightColor"] = "";
		m_MaterialSetting["LightAmbient"] = "";
		m_MaterialSetting["LightDiffuse"] = "";
		m_MaterialSetting["LightSpecular"] = "";
		m_MaterialSetting["CameraName"] = "main";
		m_MaterialSetting["TextureUnit0"] = "";
		m_MaterialSetting["TextureUnit1"] = "";
		m_MaterialSetting["TextureUnit2"] = "";
		m_MaterialSetting["TextureUnit3"] = "";
		m_MaterialSetting["TextureUnit4"] = "";
	}

	void OnMaterialSetted(TATRenderUnit* unit) //called in SetMaterial(a)
	{
		m_RenderUnits.push_back(unit);
	}

	virtual void Load(const TString& name)
	{
		__super::Load(name);

		TString path = TATPaths::PathOfMaterial(TATApplication::Instance()->GetAppName(), name);
		std::vector<TString> strs;
		TATFileStream::ReadFile(path, strs);

		for (int i = 0; i < (int)strs.size(); i++)
		{
			std::vector<TString> pair = strs[i].Split(":");

			if (m_MaterialSetting.find(pair[0]) != m_MaterialSetting.end())
			{
				m_MaterialSetting[pair[0]] = pair[1];
			}
		}

		m_Shader = new TATShader(m_MaterialSetting["VertexShader"].ToChar(), m_MaterialSetting["FragmentShader"].ToChar());
		if (m_MaterialSetting["LightName"] != "")
			m_Light = TATWorld::Instance()->GetLight("light_" + m_MaterialSetting["VertexShader"]);
		if (m_Light)
		{
			m_LightColor = m_Light->GetColor();
			m_LightAmbient = m_Light->GetAmbient();
			m_LightDiffuse = m_Light->GetDiffuse();
			m_LightSpecular = m_Light->GetSpecular();
		}
		if (m_MaterialSetting["LightColor"] != "")
			m_LightColor = m_MaterialSetting["LightColor"].ToVector3();
		if (m_MaterialSetting["LightAmbient"] != "")
			m_LightAmbient = m_MaterialSetting["LightAmbient"].ToVector3();
		if (m_MaterialSetting["LightDiffuse"] != "")
			m_LightDiffuse = m_MaterialSetting["LightDiffuse"].ToVector3();
		if (m_MaterialSetting["LightSpecular"] != "")
			m_LightSpecular = m_MaterialSetting["LightSpecular"].ToVector3();

		if (m_MaterialSetting["CameraName"] != "")
		{
			m_Camera = TATWorld::Instance()->GetCamera(m_MaterialSetting["CameraName"]);
		}

	}

	//hold the parameter that can be set from outter .tmaterial file
	//=======================
	TATVector3 m_LightColor;
	TATVector3 m_LightAmbient;
	TATVector3 m_LightDiffuse;
	TATVector3 m_LightSpecular;

	TATShader* m_Shader;
	TATLight* m_Light;
	TATCamera* m_Camera;
	TATexture* m_Textures[5];

	//=======================

	std::vector<TATRenderUnit*> m_RenderUnits; //TODO batch

	std::map<TString, TString> m_MaterialSetting;

};