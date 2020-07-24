#ifndef THEALMIGHTY_TEXTURE
#define THEALMIGHTY_TEXTURE
#include <iostream>

#include "TATGLHeader.h"

using namespace std;
class TATexture
{
public:
	TATexture()
	{
		m_Data = 0;

	}

	~TATexture()
	{
		UnLoad();
		glDeleteTextures(1, &m_GLId);
	}

	void Init(string filePath)
	{
		stbi_set_flip_vertically_on_load(true);
		Load(filePath);
		glGenTextures(1, &m_GLId);
	}

	void Use()
	{
		glBindTexture(GL_TEXTURE_2D, m_GLId);
		// set the texture wrapping parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		// set texture filtering parameters
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		if (m_Data)
		{
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, m_Width, m_Height, 0, GL_RGB, GL_UNSIGNED_BYTE, m_Data);
			glGenerateMipmap(GL_TEXTURE_2D);
		}
	}
	void Load(string filePath)
	{
		m_Data = stbi_load(filePath.c_str(), &m_Width, &m_Height, &m_ChannelNum, 0);
	}
	void UnLoad()
	{
		stbi_image_free(m_Data);
	}
	int m_Width;
	int m_Height;
	int m_ChannelNum;

	unsigned char* m_Data;
	unsigned int m_GLId;
	string m_FilePath;
};


#endif // !THEALMIGHTY_TEXTURE
