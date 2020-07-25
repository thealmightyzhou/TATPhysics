#ifndef THEALMIGHTY_GLUTILITY
#define THEALMIGHTY_GLUTILITY

#include "TATGLHeader.h"
#include "../TATCommon/TATSingleton.h"

#include <map>

using namespace std;

class TATInputListener;

class TATGLEntry:public Singleton<TATGLEntry>
{
public:
	TATGLEntry()
	{
		
	}

	GLFWwindow* Initialize(int width,int height)
	{
		m_WindowWidth = width;
		m_WindowHeight = height;

		glfwInit();
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

		// glfw window creation
		// --------------------
		GLFWwindow* window = glfwCreateWindow(width, height, "", NULL, NULL);
		if (window == NULL)
		{
			std::cout << "Failed to create GLFW window" << std::endl;
			glfwTerminate();
			return NULL;
		}
		glfwMakeContextCurrent(window);
		glfwSetFramebufferSizeCallback(window, &this->OnFrameBufferResizeCallback);
		glfwSetCursorPosCallback(window, &this->OnCursorMoveCallback);
		glfwSetScrollCallback(window, &this->OnMouseScrollCallback);
		glfwSetMouseButtonCallback(window, &this->OnMousePressedCallback);

		// tell GLFW to capture our mouse
		glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
		// glad: load all OpenGL function pointers
		// ---------------------------------------
		if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		{
			std::cout << "Failed to initialize GLAD" << std::endl;
			return NULL;
		}

		// configure global opengl state
		// -----------------------------
		glEnable(GL_DEPTH_TEST);

		// build and compile our shader zprogram
		// ------------------------------------

		return window;
	}

	void OnAppExit()
	{
		glfwTerminate();
	}

	// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
	// ---------------------------------------------------------------------------------------------------------
	virtual void processInput(GLFWwindow* window);

	void OnFrameBufferResizeCallback(GLFWwindow* window, int width, int height)
	{
		glViewport(0, 0, width, height);
	}
	// glfw: whenever the mouse moves, this callback is called
	// -------------------------------------------------------
	void OnCursorMoveCallback(GLFWwindow* window, double xpos, double ypos)
	{
		if (m_FirstCursor)
		{
			m_CursorLastX = xpos;
			m_CursorLastY = ypos;
			m_FirstCursor = false;
		}

		OnCursorMove(xpos - m_CursorLastX, m_CursorLastY - ypos);

		m_CursorLastX = xpos;
		m_CursorLastY = ypos;
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


	// glfw: whenever the mouse scroll wheel scrolls, this callback is called
	// ----------------------------------------------------------------------
	void OnMouseScrollCallback(GLFWwindow* window, double xoffset, double yoffset)
	{
		OnCursorScroll(yoffset);
	}

	void AddInputListener(TATInputListener* listener);

	void RemoveInputListener(TATInputListener* listener);

private:
	void OnKeyPressed(int key);

	void OnKeyReleased(int key);

	void OnKeyHold(int key);

	void OnCursorMove(float dx, float dy);

	void OnCursorPressed(int key);

	void OnCursorReleased(int key);

	void OnCursorHold(int key);

	void OnCursorScroll(float offset);

	std::map<void*, TATInputListener*> m_InputListeners;

	float m_CursorLastX;
	float m_CursorLastY;
	bool m_FirstCursor;
	bool m_IsMouseDown;

	int m_WindowWidth;
	int m_WindowHeight;
};

#endif // !THEALMIGHTY_GLUTILITY