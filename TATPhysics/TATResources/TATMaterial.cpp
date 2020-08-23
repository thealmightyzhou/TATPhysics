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

enum BlockType
{
	LightBlock,

};

void TATMaterial::Load(const TString& path)
{
	__super::Load(path);

	std::vector<TString> strs;
	TATFileStream::ReadFileToLines(path, strs);

	bool startBlock = false;
	std::vector<TString> blockDatas;
	std::vector<TString> blockTitle;
	BlockType type;

	for (int i = 0; i < (int)strs.size(); i++)
	{
		if (strs[i] == "")
			continue;

		if (strs[i].Contains("$"))
		{
			//start block
			if (!startBlock)
			{
				startBlock = true;
				blockDatas.push_back(strs[i]);
				strs[i].Split(" ", blockTitle);
				if (blockTitle[0] == "$Point_Light" || blockTitle[0] == "$Direct_Light")
				{
					type = LightBlock;
				}
				continue;
			}
			//end of a block
			else
			{
				startBlock = false;
				blockDatas.push_back(strs[i]);
				if (type == LightBlock)
				{
					TATLightFactory lightFactory;
					m_Lights.push_back(lightFactory.GenerateLight(blockDatas, blockDatas.size()));
				}

				blockDatas.clear();
				continue;
			}
		}

		if (startBlock)
		{
			blockDatas.push_back(strs[i]);
			continue;
		}

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


TATLight* TATLightFactory::GenerateLight(std::vector<TString>& datas, int size)
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
		lineData.clear();
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