#pragma once
#include "TATApplication.h"
#include "TATWorldListener.h"
#include "../TATGLRender/TATCamera.h"
#include "../TATResources/TATPaths.h"

class TestApp :public TATApplication,TATRenderListener,TATPhysicListener
{
public:
	TestApp(const TString& name) :TATApplication(name)
	{

	}

	virtual void Initialize()
	{
		__super::Initialize();

		TATGLEntry::Instance()->Initialize(1080, 720);

		TATResourceManager::Instance()->LoadMesh("test.obj");

		//TODO Load resources;

		m_MainCamera = new TATCamera();
	}

	virtual void CreateScene()
	{
		__super::CreateScene();

		//TODO put resources to world and set physic behavior coefficient
	}

	virtual void BeginRenderOneFrame(float dt) override
	{
		m_MainCamera->Move(dt);
	}

	virtual void RenderOneFrameEnd(float dt) override
	{}

	virtual void SimulationStart(float dt) override
	{}

	virtual void SimulationEnd(float dt) override
	{}

	TATCamera* m_MainCamera;
};