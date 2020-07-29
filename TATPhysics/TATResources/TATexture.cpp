#include "TATexture.h"

#include "../../ThirdParty/stb_image/stb_image.h"

void TATexture::Load(const TString& name)
{
	TString filePath = TATPaths::PathOfTexture(TAT_APPNAME, name);
	stbi_set_flip_vertically_on_load(true);
	InternalLoad(filePath);
	glGenTextures(1, &m_GLId);
}

void TATexture::UnLoad()
{
	stbi_image_free(m_Data);
	m_Data = 0;
	glDeleteTextures(1, &m_GLId);
}

void TATexture::InternalLoad(const TString& filePath)
{
	m_Data = stbi_load(filePath.m_Str.c_str(), &m_Width, &m_Height, &m_ChannelNum, 0);
}