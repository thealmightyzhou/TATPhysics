#include "TATWorldListener.h"
#include "TAThread.h"
#include "TATApplication.h"

TATRenderListener::TATRenderListener()
{
	TAT_RENDER_THREAD->AddListener(this);
}

TATPhysicListener::TATPhysicListener()
{
	TAT_PHYSIC_THREAD->AddListener(this);
}