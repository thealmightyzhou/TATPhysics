#pragma once
#include "../TATCommon/TATSingleton.h"
#include "../TATResources/TATPaths.h"
#include "TATWorldListener.h"

class TAThread;
class TATStageNode;
class TATCamera;
class TATLight;
class TATPhysicThread;
class TATRenderThread;
class TATRenderTaskList;

#define TAT_ROOTNODE TATApplicationEntry::Instance()->GetApplication()->GetRootNode()
#define TAT_RENDER_THREAD TATApplicationEntry::Instance()->GetApplication()->m_RenderThread
#define TAT_PHYSIC_THREAD TATApplicationEntry::Instance()->GetApplication()->m_PhysicThread
#define TAT_APPNAME TATApplicationEntry::Instance()->GetApplication()->GetAppName()
#define TAT_APP TATApplicationEntry::Instance()->GetApplication()
#define TAT_RENDER_TASK_LIST (&TAT_RENDER_THREAD->m_RenderTaskList)

class TATApplication:public TATRenderListener,public TATPhysicListener
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

	virtual void BeginRenderOneFrame(float dt) 
	{
		Update(dt);
	}

	virtual void RenderOneFrameEnd(float dt) {}

	virtual void SimulationStart(float dt) {}

	virtual void SimulationEnd(float dt) {}

	virtual void Update(float dt) {}

	TString m_AppName;

	TATPhysicThread* m_PhysicThread;

	TATRenderThread* m_RenderThread;

	TATStageNode* m_RootNode;

	TATCamera* m_MainCamera;
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

	void SetApplication(TATApplication* app)
	{
		m_App = app;
	}

	TATApplication* GetApplication()
	{
		return m_App;
	}

	static TATRenderThread* GetRenderThread();

	static TATPhysicThread* GetPhysicThread();

	TATApplication* m_App;

	static TATApplicationEntry* m_Entry;

protected:
	TATApplicationEntry()
	{
		m_App = 0;
	}
};