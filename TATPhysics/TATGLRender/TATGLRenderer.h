#ifndef THEALMIGHTY_RENDERER
#define THEALMIGHTY_RENDERER

#include "TATGLHeader.h"
#include "../TATResources/TATShader.h"
#include "TATRenderunit.h"
#include "../TATResources/TATMaterial.h"
#include "../TATBasis/TATWorld.h"
#include "TATLight.h"
#include "TATCamera.h"

using namespace std;

class TATGLRenderer//:public TATRenderer
{
public:
	TATGLRenderer()
	{
		m_RenderMode = GL_TRIANGLES;
	}

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

		if (unit->m_RenderEleMask.IsUseTexCoordinate())
		{
			unit->m_Material->m_Shader->Use();
			for (int i = 0; i < unit->m_TexCount; i++)
			{
				unit->m_Textures[i]->Generate();
				unit->m_Material->m_Shader->SetInt(TString("texture") + TString::ConvertInt(i), i);
				glActiveTexture(GLTextureSeman[i]);
				unit->m_Textures[i]->Use();
			}
		}

		glBindVertexArray(0);
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		unit->m_StaticDataUploaded = true;
	}

	//this will be called every frame marked dirty
	virtual void UploadDynamicData(TATRenderUnit* unit)
	{
		TATShader* shader = unit->m_Material->m_Shader;
		TATMaterial* material = unit->m_Material;
		float mat[16];

		shader->Use();

		material->m_Camera->GetProjectionMatrix(mat);
		shader->SetMat4("projection", unit->m_MatrixProj);

		material->m_Camera->GetViewMatrix(mat);
		shader->SetMat4("view", mat);

		if (unit->m_UseTransform)
			unit->m_Transform.GetOpenGLMatrix(mat);
		else
			TATransform::GetIdentity().GetOpenGLMatrix(mat);
		shader->SetMat4("model", mat);

		shader->SetVec3("lightPos", material->m_Light->GetPosition());

		shader->SetVec3("viewPos", material->m_Camera->GetPosition());

		shader->SetVec3("lightColor", material->m_LightColor);
	}

	virtual void Draw(TATRenderUnit* unit)
	{
		glBindVertexArray(unit->m_VAOId);

		if (m_RenderMode == GL_TRIANGLES)
		{
			glDrawArrays(GL_TRIANGLES, 0, unit->m_IndicesCount);
		}
		else if (m_RenderMode == GL_POINTS)
		{
			glDrawArrays(GL_POINTS, 0, unit->m_VertexCount);
		}

		//else if (m_renderMode == GL_LINE_LOOP || m_renderMode == GL_LINE_STRIP || m_renderMode == GL_LINES)
		//{
		//	glDrawArrays(GL_LINES, 0, 6 * m_mesh->m_Triangles.size() + 12 * m_mesh->m_Tetras.size());
		//}
		//else if (m_renderMode == GL_LINE)
		//{
		//	glDrawArrays(GL_LINES, 0, m_mesh->m_Edges.size() * 2);
		//}

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

	//GL_TRIANGLES | GL_LINES | GL_POINTS
	void SetRenderMode(UINT mode)
	{
		m_RenderMode = mode;
	}

	UINT m_RenderMode;
};

#endif // !THEALMIGHTY_RENDERER