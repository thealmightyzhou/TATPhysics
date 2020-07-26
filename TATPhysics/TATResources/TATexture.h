#ifndef THEALMIGHTY_TEXTURE
#define THEALMIGHTY_TEXTURE
#include <iostream>
#include "TATResourcePrimitive.h"
#include "../TATGLRender/TATGLHeader.h"

using namespace std;
class TATexture:public TATResourcePrimitive
{
public:
	TATexture():TATResourcePrimitive("texture_nameless" + GetObjectIndex())
	{
		m_Data = 0;
	}

	TATexture(const TString& name):TATResourcePrimitive("texture_" + name)
	{
		m_Data = 0;
	}

	~TATexture()
	{
		UnLoad();
	}

	virtual void Load(const TString& name)
	{
		TString filePath = TATPaths::PathOfTexture(TATApplication::Instance()->GetAppName(), name);
		stbi_set_flip_vertically_on_load(true);
		InternalLoad(filePath);
		glGenTextures(1, &m_GLId);
	}

	void Generate()
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

	void Use()
	{
		glBindTexture(GL_TEXTURE_2D, m_GLId);
	}

	void UnLoad()
	{
		stbi_image_free(m_Data);
		m_Data = 0;
		glDeleteTextures(1, &m_GLId);
	}

protected:
	virtual void InternalLoad(const TString& filePath)
	{
		m_Data = stbi_load(filePath.m_Str.c_str(), &m_Width, &m_Height, &m_ChannelNum, 0);
	}

	int m_Width;
	int m_Height;
	int m_ChannelNum;

	unsigned char* m_Data;
	unsigned int m_GLId;
	string m_FilePath;
};


#endif // !THEALMIGHTY_TEXTURE
