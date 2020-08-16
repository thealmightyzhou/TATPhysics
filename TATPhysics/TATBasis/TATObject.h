#pragma once
#include "../TATResources/TATPaths.h"
#include "../TATCommon/TATCore.h"

//this class will automatically add to TATWorld's object list
//will be implemented by objects like mesh,material,light,camera,texture..

class TATObject
{
public:
	TATObject(const TString& name);

	virtual ~TATObject();

	TAT_REGISTER_ATTRIBUTE_GET(TString, GlobalName);

	//for none param construct to avoid confict
	static int m_ObjectIndex;

	void Initialize();

	int GetObjectIndex()
	{
		return m_ObjectIndex++;
	}

	TString GetSubName() const
	{
		std::vector<TString> rec;
		m_GlobalName.Split("_", rec);
		return rec[rec.size() - 1];
	}

	template<typename T>
	T* Cast()
	{
		return dynamic_cast<T*>(this);
	}
};