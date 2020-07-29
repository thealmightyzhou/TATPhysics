#pragma once
#include "TATApplication.h"
#include "TATWorldListener.h"

class TestApp :public TATApplication
{
public:
	TestApp(const TString& name) :TATApplication(name)
	{}

	virtual void Initialize() override;

	virtual void CreateScene() override;

	virtual void Run() override;
};