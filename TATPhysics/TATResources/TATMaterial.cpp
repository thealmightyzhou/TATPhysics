#include "TATMaterial.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "../TATBasis/TATObject.h"
#include "../TATBasis/TATWorld.h"
#include "TATShader.h"
#include "TATPaths.h"
#include "TATFileStream.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATGLRender/TATLight.h"

void TATMaterial::Load(const TString& name)
{
	__super::Load(name);

	bool isDefault = false;
	TString path = TATPaths::PathOfMaterial(TAT_APPNAME, name);
	std::vector<TString> strs;
	TATFileStream::ReadFileToLines(path, strs);
	if (strs.size() == 0)
	{
		path = TATPaths::PathOfDefaultMaterial(name);
		TATFileStream::ReadFileToLines(path, strs);
		if (strs.size() > 0)
			isDefault = true;
	}

	for (int i = 0; i < (int)strs.size(); i++)
	{
		std::vector<TString> pair;
		strs[i].Split(":", pair);

		if (m_MaterialSetting.find(pair[0]) != m_MaterialSetting.end())
		{
			m_MaterialSetting[pair[0]] = pair[1];
		}
	}

	TString vPath, fPath, gPath;
	if (!isDefault)
	{
		vPath = TATPaths::PathOfShader(TAT_APPNAME, m_MaterialSetting["VertexShader"]);
		fPath = TATPaths::PathOfShader(TAT_APPNAME, m_MaterialSetting["FragmentShader"]);
		gPath = TATPaths::PathOfShader(TAT_APPNAME, m_MaterialSetting["GeometryShader"]);
	}
	else
	{
		vPath = TATPaths::PathOfDefaultShader(m_MaterialSetting["VertexShader"]);
		fPath = TATPaths::PathOfDefaultShader(m_MaterialSetting["FragmentShader"]);
		gPath = TATPaths::PathOfDefaultShader(m_MaterialSetting["GeometryShader"]);
	}

	m_Shader = new TATShader(vPath, fPath, gPath);

	if (m_MaterialSetting["LightName"] != "")
	{
		m_Light = TATWorld::Instance()->GetLight(m_MaterialSetting["LightName"]);
	}

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

	for (int i = 0; i < 5; i++)
	{
		if (m_MaterialSetting[TString("TextureUnit") + TString::ConvertInt(i)] != "")
		{
			m_Textures[i] = new TATexture(m_MaterialSetting[TString("TextureUnit") + TString::ConvertInt(i)].FetchInnerName());
			m_Textures[i]->Load(m_MaterialSetting[TString("TextureUnit") + TString::ConvertInt(i)]);
		}
	}

}

void TATMaterial::OnMaterialSetted(TATRenderUnit* unit)
{
	m_RenderUnits.push_back(unit);
}