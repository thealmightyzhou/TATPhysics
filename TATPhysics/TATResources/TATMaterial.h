#pragma once
#include "../TATBasis/TString.h"
#include "../TATGLRender/TATRenderUnit.h"
#include "TATFileStream.h"
#include <iostream>
#include "../TATBasis/TATObject.h"
#include "TATShader.h"

using namespace std;

class TATMaterial:public TATObject
{
public:
	TATMaterial(const TString& name):TATObject("material_" + name)
	{}

	void OnMaterialSet() //called in SetMaterial(a)
	{

	}

	TString m_VsPath;
	TString m_FsPath;
	TATVector3* m_LightPosition;
	TATVector3* m_CameraPosition;
	TATransform* m_WorldTransform;
	TATMatrix3* m_ModelMat;
	TATMatrix3* m_ViewMat;
	TATMatrix3* m_ProjMat;

	TATShader* m_Shader;
};