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

//use '$' as a block start and end
class TATMaterial :public TATResourcePrimitive
{
public:
	TATMaterial(const TString& name) : TATResourcePrimitive("material_" + name)
	{
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

	TATVector3 m_LightDirection;
	TATVector3 m_LightPosition;

	//hold the parameter that can be set from outter .tmaterial file
	//=======================
	TATVector3 m_LightColor;

	TATVector3 m_LightAmbient;
	TATVector3 m_LightDiffuse;
	TATVector3 m_LightSpecular;

	float m_LightSpecularPower;
	float m_PointLightConstant;
	float m_PointLightLinear;
	float m_PointLightExp;

	TATShader* m_Shader;
	std::vector<TATLight*> m_Lights;
	TATCamera* m_Camera;
	TATexture* m_Textures[5];

	//=======================

	std::vector<TATRenderUnit*> m_RenderUnits; //TODO batch

	std::map<TString, TString> m_MaterialSetting;
};

struct TMaterialItem
{
public:
	TMaterialItem()
	{
		m_Evaluated = false;
		m_Data = "";
	}

	void SetData(const TString& data)
	{
		m_Data = data;
		m_Evaluated = true;
	}

	bool HasEvaluated() const
	{
		return m_Evaluated;
	}

	bool m_Evaluated;
	TString m_Data;
};

class TATLightFactory
{
public:
	TATLightFactory()
	{
		TMaterialItem item;
		m_LightAttributes["position"] = item;
		m_LightAttributes["direction"] = item;
		m_LightAttributes["factor"] = item;
		m_LightAttributes["ambient"] = item;
		m_LightAttributes["diffuse"] = item;
		m_LightAttributes["specular"] = item;
	}

	TATLight* GenerateLight(std::vector<TString>& datas, int size);
	

	std::map<TString, TMaterialItem> m_LightAttributes;
};