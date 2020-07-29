#ifndef THEALMIGHTY_TEXTURE
#define THEALMIGHTY_TEXTURE
#include <iostream>
#include "TATResourcePrimitive.h"
#include "../TATGLRender/TATGLHeader.h"
#include "../TATApplication/TATApplication.h"

using namespace std;
class TATexture:public TATResourcePrimitive
{
public:
	TATexture():TATResourcePrimitive("texture_nameless" + GetObjectIndex())
	{
		m_Data = 0;
		m_GLId = TAT_SHADERID_UNUSE;
	}

	TATexture(const TString& name):TATResourcePrimitive("texture_" + name)
	{
		m_Data = 0;
		m_GLId = TAT_SHADERID_UNUSE;
	}

	~TATexture()
	{
		UnLoad();
	}

	virtual void Load(const TString& name);

	UINT Generate()
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

		glBindTexture(GL_TEXTURE_2D, 0);

		return m_GLId;
	}

	UINT GetGLId() const
	{
		return m_GLId;
	}

	void Use()
	{
		glBindTexture(GL_TEXTURE_2D, m_GLId);
	}

	void UnLoad();

protected:
	virtual void InternalLoad(const TString& filePath);

	int m_Width;
	int m_Height;
	int m_ChannelNum;

	unsigned char* m_Data;
	unsigned int m_GLId;
	string m_FilePath;
};


#endif // !THEALMIGHTY_TEXTURE
