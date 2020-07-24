#include "TATInputListener.h"
#include "TATGLEntry.h"

std::map<int, bool> TATInputListener::m_KeyState;

TATInputListener::TATInputListener()
{
	TATGLEntry::Instance()->AddInputListener(this);
}