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

	TATLight* GenerateLight(TString* datas, int size)
	{
		//$Point_Light point_test0
		//...
		//...
		//$

		std::vector<TString> lineData;

		for (int i = 1; i < size - 1; i++)
		{
			datas[i].Split(":", lineData);
			m_LightAttributes[lineData[0]].SetData(lineData[1]);
		}

		TString& firstLine = datas[0];
		firstLine.Split(" ", lineData);

		if (lineData[0] == "$Point_Light")
		{
			TATPointLight* light = new TATPointLight(lineData[1]);
			light->m_Ambient = m_LightAttributes["ambient"].HasEvaluated() ? m_LightAttributes["ambient"].m_Data.ToVector3() : TATVector3(0, 0, 0);
			light->m_Diffuse = m_LightAttributes["diffuse"].HasEvaluated() ? m_LightAttributes["diffuse"].m_Data.ToVector3() : TATVector3(0, 0, 0);
			light->m_Specular = m_LightAttributes["specular"].HasEvaluated() ? m_LightAttributes["specular"].m_Data.ToVector3() : TATVector3(0, 0, 0);
			TATVector3 factor = m_LightAttributes["factor"].HasEvaluated() ? m_LightAttributes["factor"].m_Data.ToVector3() : TATVector3(0, 0, 0);
			light->m_Constant = factor[0];
			light->m_Linear = factor[1];
			light->m_Exp = factor[2];
			light->m_Position = m_LightAttributes["position"].HasEvaluated() ? m_LightAttributes["position"].m_Data.ToVector3() : TATVector3(0, 0, 0);
			return light;
		}

		else if (lineData[0] == "$Direct_Light")
		{
			TATDirectionLight* light = new TATDirectionLight(lineData[1]);
			light->m_Ambient = m_LightAttributes["ambient"].HasEvaluated() ? m_LightAttributes["ambient"].m_Data.ToVector3() : TATVector3(0, 0, 0);
			light->m_Diffuse = m_LightAttributes["diffuse"].HasEvaluated() ? m_LightAttributes["diffuse"].m_Data.ToVector3() : TATVector3(0, 0, 0);
			light->m_Specular = m_LightAttributes["specular"].HasEvaluated() ? m_LightAttributes["specular"].m_Data.ToVector3() : TATVector3(0, 0, 0);
			light->m_Direction = m_LightAttributes["direction"].HasEvaluated() ? m_LightAttributes["direction"].m_Data.ToVector3() : TATVector3(0, -1, 0);

			return light;
		}
	}

	std::map<TString, TMaterialItem> m_LightAttributes;
};