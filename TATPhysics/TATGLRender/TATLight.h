#ifndef THEALMIGHTY_LIGHT
#define THEALMIGHTY_LIGHT

#include "TATGLHeader.h"
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATCore.h"

#include <vector>

enum LIGHTTYPE
{
	DIRECTIONAL_LIGHT,
	POINT_LIGHT,
	SPOT_LIGHT,
};

class TATLight
{
public:
	TATLight()
	{
		SetColor(TATVector3(1, 1, 1));
	}

	virtual ~TATLight() {}

	TAT_REGISTER_ATTRIBUTE(TATVector3, Position);
	TAT_REGISTER_ATTRIBUTE(TATVector3, Color);
	TAT_REGISTER_ATTRIBUTE(float, Ambient);
	TAT_REGISTER_ATTRIBUTE(float, Diffuse);
	TAT_REGISTER_ATTRIBUTE(float, Specular);
	TAT_REGISTER_ATTRIBUTE_GET(LIGHTTYPE, Type);
};

class TATDirectionLight :public TATLight
{
public:
	TATDirectionLight()
	{
		m_Type = DIRECTIONAL_LIGHT;
	}
	TAT_REGISTER_ATTRIBUTE(TATVector3, Direction);
};

class TATSpotLight :public TATLight
{
public:
	TATSpotLight()
	{
		m_Type = SPOT_LIGHT;
	}
	TAT_REGISTER_ATTRIBUTE(float, InnerAngle);
	TAT_REGISTER_ATTRIBUTE(float, OuterAngle);
	TAT_REGISTER_ATTRIBUTE(float, Rate);
};

class TATPointLight :public TATLight
{
public:
	TATPointLight()
	{
		m_Type = POINT_LIGHT;
	}
};

#endif//THEALMIGHTY_LIGHT
