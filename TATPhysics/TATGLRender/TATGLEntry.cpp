#include "TATGLEntry.h"
#include "TATInputListener.h"

#define ALL_LISTENERS_DO(FUNC)													\
	std::map<void*, TATInputListener*>::iterator ite = m_InputListeners.begin();\
	for (ite; ite!=m_InputListeners.end(); ite++)								\
		if (ite->second)														\
			ite->second->FUNC;

void TATGLEntry::OnKeyPressed(int key)
{
	ALL_LISTENERS_DO(OnKeyPressed(key));
	TATInputListener::m_KeyState[key] = true;
}

void TATGLEntry::OnKeyReleased(int key)
{
	ALL_LISTENERS_DO(OnKeyReleased(key));
	TATInputListener::m_KeyState[key] = false;
}

void TATGLEntry::OnKeyHold(int key)
{
	ALL_LISTENERS_DO(OnKeyHold(key));
}

void TATGLEntry::OnCursorMove(float dx, float dy)
{
	ALL_LISTENERS_DO(OnCursorMove(dx, dy));
}

void TATGLEntry::OnCursorPressed(int key)
{
	ALL_LISTENERS_DO(OnCursorPressed(key));
}

void TATGLEntry::OnCursorReleased(int key)
{
	ALL_LISTENERS_DO(OnCursorReleased(key));
}

void TATGLEntry::OnCursorHold(int key)
{
	ALL_LISTENERS_DO(OnCursorHold(key));
}

void TATGLEntry::OnCursorScroll(float offset)
{
	ALL_LISTENERS_DO(OnCursorScroll(offset));
}

#undef ALL_LISTENERS_DO(FUNC)

void TATGLEntry::AddInputListener(TATInputListener* listener)
{
	m_InputListeners.insert(std::make_pair((void*)(listener), listener));
}

void TATGLEntry::RemoveInputListener(TATInputListener* listener)
{
	m_InputListeners.erase((void*)listener);
}

void TATGLEntry::processInput(GLFWwindow* window)
{
	std::map<int, bool>::iterator ite = TATInputListener::m_KeyState.begin();
	for (ite; ite != TATInputListener::m_KeyState.end(); ite++)
	{
		if (glfwGetKey(window, ite->first) == GLFW_PRESS)
		{
			if (!ite->second)
			{
				ite->second = true;
				OnKeyPressed(ite->first);
			}
			OnKeyHold(ite->first);
		}
		else if (glfwGetKey(window, ite->first) == GLFW_RELEASE)
		{
			if (ite->second)
			{
				ite->second = false;
				OnKeyReleased(ite->first);
			}
		}
	}
}