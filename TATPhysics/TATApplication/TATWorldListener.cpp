#include "TATWorldListener.h"
#include "TAThread.h"
#include "TATApplication.h"

TATRenderListener::TATRenderListener()
{
	if(TAT_APP && TAT_RENDER_THREAD)
		TAT_RENDER_THREAD->AddListener(this);
}

TATPhysicListener::TATPhysicListener()
{
	if(TAT_APP && TAT_PHYSIC_THREAD)
		TAT_PHYSIC_THREAD->AddListener(this);
}