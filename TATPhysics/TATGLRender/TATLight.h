#ifndef THEALMIGHTY_LIGHT
#define THEALMIGHTY_LIGHT

#include "TATGLHeader.h"
#include "../TATCommon/TATVector3.h"
#include "../TATCommon/TATCore.h"
#include "../TATBasis/TString.h"
#include "../TATBasis/TATObject.h"
#include <vector>

enum LIGHTTYPE
{
	DIRECTIONAL_LIGHT,
	POINT_LIGHT,
	SPOT_LIGHT,
};

class TATLight:public TATObject
{
public:
	TATLight(const TString& name):TATObject("light_" + name)
	{
		m_Color = TATVector3::One();
		m_Position = TATVector3::Zero();
	}

	virtual ~TATLight() {}

	TAT_REGISTER_ATTRIBUTE(TATVector3, Position);
	TAT_REGISTER_ATTRIBUTE(TATVector3, Color);
	TAT_REGISTER_ATTRIBUTE(TATVector3, Ambient);
	TAT_REGISTER_ATTRIBUTE(TATVector3, Diffuse);
	TAT_REGISTER_ATTRIBUTE(TATVector3, Specular);
	TAT_REGISTER_ATTRIBUTE_GET(LIGHTTYPE, Type);
};

class TATDirectionLight :public TATLight
{
public:
	TATDirectionLight(const TString& name):TATLight(name)
	{
		m_Type = DIRECTIONAL_LIGHT;
	}
	TAT_REGISTER_ATTRIBUTE(TATVector3, Direction);
};

class TATSpotLight :public TATLight
{
public:
	TATSpotLight(const TString& name):TATLight(name)
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
	TATPointLight(const TString& name):TATLight(name)
	{
		m_Type = POINT_LIGHT;
		m_Constant = 1.0f;
		m_Linear = 1.0f;
		m_Exp = 0.5f;
	}

	TAT_REGISTER_ATTRIBUTE(TATVector3, Position);
	TAT_REGISTER_ATTRIBUTE(float, Constant);
	TAT_REGISTER_ATTRIBUTE(float, Linear);
	TAT_REGISTER_ATTRIBUTE(float, Exp);
};

#endif//THEALMIGHTY_LIGHT
