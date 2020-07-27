#include "TATGLEntry.h"
#include "TATInputListener.h"

static TATGLEntry* global_GLEntry = new TATGLEntry;

TATGLEntry* TATGLEntry::m_Instance = global_GLEntry;

void FrameBufferResizeWrapper(GLFWwindow* window, int width, int height)
{
	if(global_GLEntry)
		global_GLEntry->OnFrameBufferResizeCallback(global_GLEntry->GetWindow(), width, height);
}

void CursorMoveCallbackWrapper(GLFWwindow* window, double xpos, double ypos)
{
	if(global_GLEntry)
		global_GLEntry->OnCursorMoveCallback(global_GLEntry->GetWindow(), xpos, ypos);
}

void MouseScrollCallbackWrapper(GLFWwindow* window, double xoffset, double yoffset)
{
	if(global_GLEntry)
		global_GLEntry->OnMouseScrollCallback(global_GLEntry->GetWindow(), xoffset, yoffset);
}

void MousePressedCallbackWrapper(GLFWwindow* window, int button, int action, int mods)
{
	if(global_GLEntry)
		global_GLEntry->OnMousePressedCallback(global_GLEntry->GetWindow(), button, action, mods);
}

GLFWwindow* TATGLEntry::Initialize(int width, int height)
{
	m_Instance = global_GLEntry;

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

	glfwSetFramebufferSizeCallback(window, FrameBufferResizeWrapper);
	glfwSetCursorPosCallback(window, CursorMoveCallbackWrapper);
	glfwSetScrollCallback(window, MouseScrollCallbackWrapper);
	glfwSetMouseButtonCallback(window, MousePressedCallbackWrapper);

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