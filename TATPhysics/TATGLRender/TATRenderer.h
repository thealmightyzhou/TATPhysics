#ifndef THEALMIGHTY_RENDERER
#define THEALMIGHTY_RENDERER

#include "TATGLHeader.h"
#include "TATShader.h"
#include "TATexture.h"
#include "TATCamera.h"
#include "TATVertexFactory.h"

using namespace std;

class TATRenderer
{
public:
	TATRenderer(Mesh *mesh, Camera *cam) :m_mesh(mesh), m_cam(cam)
	{
		m_usePosition = false;
		m_useTexcoord = false;
		m_useNormal = false;
		m_useTexNum = 0;
		m_renderMode = GL_TRIANGLES;
		m_modelMat = glm::mat4(1.0f);
	}
	TATRenderer(Camera *cam) :m_cam(cam)
	{
		m_mesh = 0;
		m_usePosition = false;
		m_useTexcoord = false;
		m_useNormal = false;
		m_useTexNum = 0;
		m_renderMode = GL_TRIANGLES;
		m_modelMat = glm::mat4(1.0f);
	}

	virtual~Renderer(){}
	virtual void Preset(Shader* shader)
	{
		m_shader = shader;
		CalcDataOffset();
		const std::vector<Vert3D, Eigen::aligned_allocator<Vector3d>> &verts = m_mesh->m_Verts;
		const std::vector<Triangle3D, Eigen::aligned_allocator<Vector3d>> &tris = m_mesh->m_Triangles;
		const std::vector<Edge3D, Eigen::aligned_allocator<Vector3d>> &edges = m_mesh->m_Edges;
		if (m_renderMode == GL_TRIANGLES || m_renderMode == GL_TRIANGLE_STRIP || m_renderMode == GL_TRIANGLE_FAN)
		{
			if (tris.size() == 0)
			{
				std::cout << "no tris to render!";
				return;
			}
			for (int i = 0; i < tris.size(); i++)
			{
				EigenToNormalVec3(tris[i].m_p1.m_Pos, m_renderVerts);
				if (m_useNormal)
					EigenToNormalVec3(tris[i].m_p1.m_Normal, m_renderVerts);
				EigenToNormalVec3(tris[i].m_p2.m_Pos, m_renderVerts);
				if (m_useNormal)
					EigenToNormalVec3(tris[i].m_p2.m_Normal, m_renderVerts);
				EigenToNormalVec3(tris[i].m_p3.m_Pos, m_renderVerts);
				if (m_useNormal)
					EigenToNormalVec3(tris[i].m_p3.m_Normal, m_renderVerts);
			}
		}
		else if (m_renderMode == GL_POINTS)
		{
			if (verts.size() == 0)
			{
				std::cout << "no verts to render!";
				return;
			}
			for (int i = 0; i < verts.size(); i++)
			{
				EigenToNormalVec3(verts[i].m_Pos, m_renderVerts);
				if (m_useNormal)
					EigenToNormalVec3(verts[i].m_Normal, m_renderVerts);
			}
		}
		else if (m_renderMode == GL_LINE || m_renderMode == GL_LINE_LOOP || m_renderMode == GL_LINE_LOOP || m_renderMode == GL_LINES)
		{
			if (edges.size() == 0)
			{
				m_mesh->BuildEdges();
				std::cout << "no edges to render!";
				return;
			}
			for (int i = 0; i < edges.size(); i++)
			{
				EigenToNormalVec3(edges[i].m_p1.m_Pos, m_renderVerts);
				if (m_useNormal)
					EigenToNormalVec3(edges[i].m_p1.m_Normal, m_renderVerts);
				EigenToNormalVec3(edges[i].m_p2.m_Pos, m_renderVerts);
				if (m_useNormal)
					EigenToNormalVec3(edges[i].m_p2.m_Normal, m_renderVerts);
			}
		}
		if (VAO == 0)
			glGenVertexArrays(1, &VAO);
		if (VBO == 0)
			glGenBuffers(1, &VBO);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, m_renderVerts.size()*sizeof(float), &m_renderVerts[0], GL_STATIC_DRAW);

		//attributes
		if (m_usePosition)
		{
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, m_totalSize * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);
		}
		else
			return;

		if (m_useTexcoord)
		{
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, m_totalSize * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(1);
			if (m_useNormal)
			{
				glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, m_totalSize * sizeof(float), (void*)(5 * sizeof(float)));
				glEnableVertexAttribArray(2);
			}
		}
		else
		{
			if (m_useNormal)
			{
				glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, m_totalSize * sizeof(float), (void*)(3 * sizeof(float)));
				glEnableVertexAttribArray(1);
			}
		}

		if (m_useTexNum > 0)
		{
			tex1.Use();
			shader->use();
			shader->setInt("texture1", 0);
			if (m_useTexNum > 1)
			{
				tex2.Use();
				shader->setInt("texture2", 1);
			}
			
		}

		glBindVertexArray(0);

	}

	virtual void Preset(Shader* shader, 
		const std::vector<Vert3D, Eigen::aligned_allocator<Vector3d>> &Verts,
		const std::vector<Triangle3D, Eigen::aligned_allocator<Vector3d>> &Tris,
		const std::vector<Edge3D, Eigen::aligned_allocator<Vector3d>> &Edges)
	{
		m_shader = shader;
		CalcDataOffset();
		const std::vector<Vert3D, Eigen::aligned_allocator<Vector3d>> &verts = Verts;
		const std::vector<Triangle3D, Eigen::aligned_allocator<Vector3d>> &tris = Tris;
		const std::vector<Edge3D, Eigen::aligned_allocator<Vector3d>> &edges = Edges;
		if (m_renderMode == GL_TRIANGLES || m_renderMode == GL_TRIANGLE_STRIP || m_renderMode == GL_TRIANGLE_FAN)
		{
			if (tris.size() == 0)
			{
				std::cout << "no tris to render!";
				return;
			}
			for (int i = 0; i < tris.size(); i++)
			{
				EigenToNormalVec3(tris[i].m_p1.m_Pos, m_renderVerts);
				EigenToNormalVec3(tris[i].m_p2.m_Pos, m_renderVerts);
				EigenToNormalVec3(tris[i].m_p3.m_Pos, m_renderVerts);
			}
		}
		else if (m_renderMode == GL_POINTS)
		{
			if (verts.size() == 0)
			{
				std::cout << "no verts to render!";
				return;
			}
			for (int i = 0; i < verts.size(); i++)
			{
				EigenToNormalVec3(verts[i].m_Pos, m_renderVerts);
			}
		}
		else if (m_renderMode == GL_LINE || m_renderMode == GL_LINE_LOOP || GL_LINE_STRIP || m_renderMode == GL_LINES)
		{
			if (edges.size() == 0)
			{
				std::cout << "no edges to render!";
				return;
			}
			for (int i = 0; i < edges.size(); i++)
			{
				EigenToNormalVec3(edges[i].m_p1.m_Pos, m_renderVerts);
				EigenToNormalVec3(edges[i].m_p2.m_Pos, m_renderVerts);
			}
		}
		if (VAO == 0)
			glGenVertexArrays(1, &VAO);
		if (VBO == 0)
			glGenBuffers(1, &VBO);

		glBindVertexArray(VAO);

		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, m_renderVerts.size()*sizeof(float), &m_renderVerts[0], GL_STATIC_DRAW);

		//attributes
		if (m_usePosition)
		{
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, m_totalSize * sizeof(float), (void*)0);
			glEnableVertexAttribArray(0);
		}
		else
			return;

		if (m_useTexcoord)
		{
			glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, m_totalSize * sizeof(float), (void*)(3 * sizeof(float)));
			glEnableVertexAttribArray(1);
			if (m_useNormal)
			{
				glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, m_totalSize * sizeof(float), (void*)(5 * sizeof(float)));
				glEnableVertexAttribArray(2);
			}
		}
		else
		{
			if (m_useNormal)
			{
				glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, m_totalSize * sizeof(float), (void*)(3 * sizeof(float)));
				glEnableVertexAttribArray(1);
			}
		}

		if (m_useTexNum > 0)
		{
			tex1.Use();
			shader->use();
			shader->setInt("texture1", 0);
			if (m_useTexNum > 1)
			{
				tex2.Use();
				shader->setInt("texture2", 1);
			}

		}

		glBindVertexArray(0);

	}

	 virtual void Render()
	{
		if (m_useTexNum > 0)
		{
			glActiveTexture(GL_TEXTURE0);
			glBindTexture(GL_TEXTURE_2D, tex1.m_glId);
			if (m_useTexNum > 1)
			{
				glActiveTexture(GL_TEXTURE1);
				glBindTexture(GL_TEXTURE_2D, tex2.m_glId);
			}
		}

		m_shader->use();

		m_shader->setMat4("projection", m_cam->GetProjectionMatrix());

		m_shader->setMat4("view", m_cam->GetViewMatrix());

		m_shader->setMat4("model", m_modelMat);

		m_shader->setVec3("lightPos", World::Instance()->GetLight(0)->m_pos);

		m_shader->setVec3("viewPos", World::Instance()->GetMainCamera()->m_position);

		m_shader->setVec3("lightColor", glm::vec3(1.0f, 1.0f, 1.0f));

		m_shader->setVec3("objectColor", m_color);

		glBindVertexArray(VAO);

		if (m_renderMode == GL_TRIANGLES || m_renderMode == GL_TRIANGLE_STRIP || m_renderMode == GL_TRIANGLE_FAN)
		{
			glDrawArrays(GL_TRIANGLES, 0, 3 * m_mesh->m_Triangles.size());
		}
		else if (m_renderMode == GL_POINTS)
		{
			glDrawArrays(GL_POINTS, 0, m_mesh->m_Verts.size());
		}
		else if (m_renderMode == GL_LINE_LOOP || m_renderMode == GL_LINE_STRIP || m_renderMode == GL_LINES)
		{
			glDrawArrays(GL_LINES, 0, 6 * m_mesh->m_Triangles.size() + 12 * m_mesh->m_Tetras.size());
		}
		else if (m_renderMode == GL_LINE)
		{
			glDrawArrays(GL_LINES, 0, m_mesh->m_Edges.size() * 2);
		}

		glBindVertexArray(0);
		m_renderVerts.clear();
	}

	void SetAttribute(bool pos,bool tex,bool nor,int texNum)
	{
		m_usePosition = pos;
		m_useTexcoord = tex;
		m_useNormal = nor;
		if (m_useTexcoord)
			m_useTexNum = texNum;
	}

	void SetColor(const glm::vec3& color){ m_color = color; }

	//GL_TRIANGLES | GL_LINES | GL_POINTS
	void SetRenderMode(int mode)
	{
		m_renderMode = mode;
	}

	void LoadTex(string filePath,int index)
	{
		if (!m_useTexcoord)
			return;
		if (index == 0)
		{
			tex1.Init(filePath);
		}
		if (index == 1)
			tex2.Init(filePath);
	}

	void CalcDataOffset()
	{
		m_totalSize = m_usePosition * 3 + m_useTexcoord * 2 + m_useNormal * 3;
	}

	void SetModelMat(const glm::mat4& modelMat)
	{
		m_modelMat = modelMat;
	}

	bool m_usePosition;
	bool m_useTexcoord;
	bool m_useNormal;
	int m_useTexNum;
	int m_renderMode;
	int m_totalSize;

	int m_TexCount;
	TATexture* m_Textures;

	Texture tex1, tex2;
	Shader* m_shader;
	Camera* m_cam;
	unsigned int VBO, VAO;
	Mesh* m_mesh;
	std::vector<float> m_renderVerts;
	glm::vec3 m_color;
	glm::mat4 m_modelMat;
};

#endif // !THEALMIGHTY_RENDERER