#pragma once
#include "TATApplication.h"

class TestApp :public TATApplication
{
public:
	TestApp(const TString& name) :TATApplication(name)
	{}

	virtual void Initialize()
	{
		__super::Initialize();

		//TATResourceManager::Instance()->LoadMesh()

		//TODO Load resources;
	}

	virtual void CreateScene()
	{
		__super::CreateScene();

		//TODO put resources to world and set physic behavior coefficient
	}
};