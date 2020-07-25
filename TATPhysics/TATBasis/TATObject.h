#pragma once
#include "TString.h"
#include "../TATCommon/TATCore.h"

//this class will automatically add to TATWorld's object list
//will be implemented by objects like mesh,material,light,camera,texture..
class TATObject
{
public:
	TATObject(const TString& name);

	virtual ~TATObject();

	TAT_REGISTER_ATTRIBUTE_GET(TString, GlobalName);

	//for none param construct so can use object pool
	static int m_ObjectIndex;

	int GetObjectIndex()
	{
		return m_ObjectIndex++;
	}
};