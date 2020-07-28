#pragma once
#include "../TATCommon/TATSingleton.h"
#include "../TATResources/TATPaths.h"

class TAThread;
class TATStageNode;
class TATCamera;
class TATLight;

#define TAT_ROOTNODE TATApplicationEntry::GetApplication()->GetRootNode()

class TATApplication
{
public:
	TATApplication(const TString& name);

	virtual ~TATApplication() {}

	virtual void Initialize();

	virtual void CreateScene() {}

	//global entry
	virtual void Run() {}

	const TString& GetAppName();

	TATStageNode* GetRootNode();

	TString m_AppName;

	TAThread* m_PhysicThread;

	TAThread* m_RenderThread;

	TATStageNode* m_RootNode;

	TATCamera* m_MainCamera;

	TATLight* m_MainLight;
};

class TATApplicationEntry
{
public:
	static TATApplicationEntry* Instance()
	{
		if (m_Entry)
			return m_Entry;
		else
		{
			m_Entry = new TATApplicationEntry;
			return m_Entry;
		}
	}

	static void SetApplication(TATApplication* app)
	{
		Instance()->m_App = app;
	}

	static TATApplication* GetApplication()
	{
		return Instance()->m_App;
	}

	TATApplication* m_App;

	static TATApplicationEntry* m_Entry;

protected:
	TATApplicationEntry()
	{
		m_App = 0;
	}
};