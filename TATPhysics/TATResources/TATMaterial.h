#pragma once
#include "../TATBasis/TString.h"
#include "../TATCommon/TATCore.h"
#include "../TATCommon/TATVector3.h"
#include "../TATResources/TATResourcePrimitive.h"
#include <iostream>
#include <map>

class TATShader;
class TATLight;
class TATCamera;
class TATRenderUnit;
class TATexture;

using namespace std;

class TATMaterial :public TATResourcePrimitive
{
public:
	TATMaterial(const TString& name) : TATResourcePrimitive("material_" + name)
	{
		m_Light = 0;
		m_Shader = 0;
		m_Camera = 0;
		TAT_MEMSET(m_Textures, 0);

		Register();
	}

	void Register()
	{
		m_MaterialSetting["VertexShader"] = "";
		m_MaterialSetting["FragmentShader"] = "";
		m_MaterialSetting["GeometryShader"] = "";
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

	void GetShaderPaths(TString& vs, TString& fs, TString& gs)
	{
		vs = m_MaterialSetting["VertexShader"];
		fs = m_MaterialSetting["FragmentShader"];
		gs = m_MaterialSetting["GeometryShader"];
	}

	//called in SetMaterial(a)
	void OnMaterialSetted(TATRenderUnit* unit);

	virtual void Load(const TString& name);

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