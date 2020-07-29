#ifndef THEALMIGHTY_GLUTILITY
#define THEALMIGHTY_GLUTILITY

#include "TATGLHeader.h"
#include <iostream>
#include <map>

using namespace std;

class TATInputListener;

class TATGLEntry
{
public:
	//use Instance() instead!
	TATGLEntry()
	{
		m_GLWindow = 0;
	}

	GLFWwindow* Initialize(int width, int height);

	void OnAppExit()
	{
		glfwTerminate();
	}

	GLFWwindow* GetWindow()
	{
		return m_GLWindow;
	}

	static TATGLEntry* Instance()
	{
		return m_Instance;
	}

	virtual void OnKeyPressedCallback(GLFWwindow* window);

	void OnFrameBufferResizeCallback(GLFWwindow* window, int width, int height)
	{
		glViewport(0, 0, width, height);
	}

	void OnCursorMoveCallback(GLFWwindow* window, double xpos, double ypos)
	{
		if (m_FirstCursor)
		{
			m_CursorLastX = (float)xpos;
			m_CursorLastY = (float)ypos;
			m_FirstCursor = false;
		}

		OnCursorMove((float)xpos - m_CursorLastX, m_CursorLastY - (float)ypos);

		m_CursorLastX = (float)xpos;
		m_CursorLastY = (float)ypos;
	}

	void OnMousePressedCallback(GLFWwindow* window, int button, int action, int mods)
	{
		if (action == GLFW_PRESS)
		{
			switch (button)
			{
			case GLFW_MOUSE_BUTTON_LEFT:
				m_IsMouseDown = true;
				break;
			case GLFW_MOUSE_BUTTON_MIDDLE:
				break;
			case GLFW_MOUSE_BUTTON_RIGHT:
				break;
			default:
				return;
			}
			OnCursorPressed(button);
		}

		else if (action == GLFW_RELEASE)
		{
			switch (button)
			{
			case GLFW_MOUSE_BUTTON_LEFT:
				m_IsMouseDown = false;
				break;
			case GLFW_MOUSE_BUTTON_MIDDLE:
				break;
			case GLFW_MOUSE_BUTTON_RIGHT:
				break;
			default:
				return;
			}
			OnCursorReleased(button);
		}

		return;
	}

	void OnMouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
	{
		OnCursorScroll((float)yoffset);
	}

	void AddInputListener(TATInputListener* listener);

	void RemoveInputListener(TATInputListener* listener);

	int m_WindowWidth;
	int m_WindowHeight;
private:

	void OnKeyPressed(int key);

	void OnKeyReleased(int key);

	void OnKeyHold(int key);

	void OnCursorMove(float dx, float dy);

	void OnCursorPressed(int key);

	void OnCursorReleased(int key);

	void OnCursorHold(int key);

	void OnCursorScroll(float offset);

	void RegisterKeys();

	std::map<void*, TATInputListener*> m_InputListeners;

	float m_CursorLastX;
	float m_CursorLastY;
	bool m_FirstCursor;
	bool m_IsMouseDown;

	GLFWwindow* m_GLWindow;

	static TATGLEntry* m_Instance;
};

#endif // !THEALMIGHTY_GLUTILITY