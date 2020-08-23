#ifndef THEALMIGHTY_RENDERER
#define THEALMIGHTY_RENDERER

#include "TATGLHeader.h"
#include "../TATResources/TATShader.h"
#include "TATRenderUnit.h"
#include "../TATResources/TATMaterial.h"
#include "../TATBasis/TATWorld.h"
#include "TATLight.h"
#include "TATCamera.h"

using namespace std;

class TATGLRenderer//:public TATRenderer
{
public:
	TATGLRenderer()
	{}

	virtual ~TATGLRenderer() {}

	//upload vertex positions and bind textures
	virtual void UploadStaticData(TATRenderUnit* unit)
	{
		if (unit->m_VAOId == TAT_SHADERID_UNUSE)
			glGenVertexArrays(1, &unit->m_VAOId);
		if (unit->m_VBOId == TAT_SHADERID_UNUSE)
			glGenBuffers(1, &unit->m_VBOId);

		UINT VAO = unit->m_VAOId;
		UINT VBO = unit->m_VBOId;

		glBindVertexArray(VAO);
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		
		glBufferData(GL_ARRAY_BUFFER, sizeof(float) * unit->m_BlockSize * unit->m_IndicesCount, unit->m_RenderBuffer, GL_STATIC_DRAW);

		for (int i = 0; i < (int)unit->m_RenderEleMask.m_BufferOffsets.size(); i++)
		{
			const TATModelElementMask::BufferOffset& offset = unit->m_RenderEleMask.m_BufferOffsets[i];
			glVertexAttribPointer(
				offset.m_Index,
				offset.m_Size,
				GL_FLOAT,
				GL_FALSE,
				unit->m_RenderEleMask.m_TotalSize * sizeof(float),
				(void*)(offset.m_BeforeSize * sizeof(float)));

			glEnableVertexAttribArray(offset.m_Index);
		}

		static UINT GLTextureSeman[5]{ 0x84C0 ,0x84C1 ,0x84C2 ,0x84C3 ,0x84C4 };

		//set sampler2d
		if (unit->m_RenderEleMask.IsUseTexCoordinate())
		{
			unit->m_Material->m_Shader->Use();
			for (int i = 0; i < unit->m_TexCount; i++)
			{
				unit->m_Textures[i]->Generate();
				unit->m_Material->m_Shader->SetInt(TString("_tex") + TString::ConvertInt(i), i);
				glActiveTexture(GLTextureSeman[i]);
				unit->m_Textures[i]->Use();
			}
		}

		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		unit->m_StaticDataUploaded = true;
	}

	virtual void Draw(TATRenderUnit* unit)
	{
		glBindVertexArray(unit->m_VAOId);

		glDrawArrays(unit->m_RenderMode, 0, unit->m_IndicesCount);

		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);
	}

	virtual void Render(TATRenderUnit* unit)
	{
		if(!unit->m_StaticDataUploaded)
			UploadStaticData(unit);

		UploadDynamicData(unit);

		Draw(unit);
	}

	//this will be called every frame marked dirty
	virtual void UploadDynamicData(TATRenderUnit* unit)
	{
		TATShader* shader = unit->m_Material->m_Shader;
		shader->Use();

		UploadMatrices(unit);

		UploadLights(unit);

		TATMaterial* material = unit->m_Material;

		shader->SetVec3(TString("_view_pos"), material->m_Camera->GetPosition());

	}

	void UploadMatrices(TATRenderUnit* unit)
	{
		TATShader* shader = unit->m_Material->m_Shader;
		TATMaterial* material = unit->m_Material;
		float mat[16];

		material->m_Camera->GetProjectionMatrix(mat);
		shader->SetMat4("projection", mat);

		material->m_Camera->GetViewMatrix(mat);
		shader->SetMat4("view", mat);

		if (unit->m_UseTransform)
			unit->m_Transform.GetOpenGLMatrix(mat);
		else
			TATransform::GetIdentity().GetOpenGLMatrix(mat);
		shader->SetMat4("model", mat);
	}

	void UploadLights(TATRenderUnit* unit)
	{
		TATShader* shader = unit->m_Material->m_Shader;
		TATMaterial* material = unit->m_Material;
		int plCount = 0;
		int dlCount = 0;
		for (int i = 0; i < material->m_Lights.size(); ++i)
		{
			if (material->m_Lights[i]->m_Type == LIGHTTYPE::DIRECTIONAL_LIGHT)
			{
				UploadDirectionLight(shader, material->m_Lights[i]->Cast<TATDirectionLight>(), dlCount);
				dlCount++;
			}
			else if (material->m_Lights[i]->m_Type == LIGHTTYPE::POINT_LIGHT)
			{
				UploadPointLight(shader, material->m_Lights[i]->Cast<TATPointLight>(), plCount);
				plCount++;
			}
		}
	}

	void UploadDirectionLight(TATShader* shader, TATDirectionLight* dl, int index)
	{
		TString lightName = TString("_direct_light") + TString::ConvertInt(index);
		shader->SetVec3(lightName + TString(".ambient"), dl->m_Ambient);
		shader->SetVec3(lightName + TString(".diffuse"), dl->m_Diffuse);
		shader->SetVec3(lightName + TString(".specular"), dl->m_Specular);
		shader->SetInt(lightName + TString(".specular_power"), dl->m_SpecularPower);
		shader->SetVec3(lightName + TString(".direction"), dl->m_Direction);

	}

	void UploadPointLight(TATShader* shader, TATPointLight* pl, int index)
	{
		TString lightName = TString("_point_light") + TString::ConvertInt(index);
		shader->SetVec3(lightName + TString(".ambient"), pl->m_Ambient);
		shader->SetVec3(lightName + TString(".diffuse"), pl->m_Diffuse);
		shader->SetVec3(lightName + TString(".specular"), pl->m_Specular);
		shader->SetInt(lightName + TString(".specular_power"), pl->m_SpecularPower);
		shader->SetVec3(lightName + TString(".position"), pl->m_Position);
		shader->SetFloat(lightName + TString(".constant"), pl->m_Constant);
		shader->SetFloat(lightName + TString(".linear"), pl->m_Linear);
		shader->SetFloat(lightName + TString(".exp"), pl->m_Exp);
	}
};

#endif // !THEALMIGHTY_RENDERER