#include "Window.h"
#include <utils.h>

static void APIENTRY MyDebugCallback(GLenum source, GLenum type, GLuint id,
	GLenum severity, GLsizei length,
	const GLchar* message, const void* userParam)
{
	syslog("%s", message);
}

CWindow::CWindow()
{
	Clear();
}

CWindow::~CWindow()
{
	Destroy();
}

void CWindow::Clear()
{
	if (m_pGLWindow)
	{
		glfwDestroyWindow(m_pGLWindow);
		m_pGLWindow = nullptr;
	}

	m_pMonitor = nullptr;
	m_pVideoMode = nullptr;

	m_iWidth = 0;
	m_iHeight = 0;

	m_iFullScreenWidth = 0;
	m_iFullScreenHeight = 0;

	m_iWindowedWidth = 0;
	m_iWindowedHeight = 0;

	m_ubWindowType = 0;

	// Timing
	m_fLastFrame = 0.0f;
	m_fDeltaTime = 0.0f;

	// Cursor Part
	m_iCurrentCursor = GLFW_ARROW_CURSOR;

	glfwDestroyCursor(m_mCursorsPtr[GLFW_ARROW_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_IBEAM_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_CROSSHAIR_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_HAND_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_HRESIZE_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_VRESIZE_CURSOR]);
	m_mCursorsPtr.clear();

	m_v2MousePos = 0.0f;
	m_fMouseScroll = 0.0f;
	m_bMouseScrollUpdate = true;

	// Input
	m_bKeyBools.fill(false);
	m_bMouseKeys.fill(false);
}

void CWindow::Destroy()
{
	Clear();
	glfwTerminate();
}

bool CWindow::InitializeWindow()
{
	if (glfwInit() == false)
	{
		syserr("Failed to Initialize GLFW");
		return (false);
	}

	// Setup OpenGL Version (4.6) and Enable core profile (Delete Deperecated functions)
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Enable Multi Sampling, for a smoother drawing
	glfwWindowHint(GLFW_SAMPLES, 4);

	// Enable Context Debugging for OpenGL
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GLFW_TRUE);

	// More Window Flags
	glfwWindowHint(GLFW_DECORATED, GLFW_TRUE); // Make it with "title bar" and "minimize, exit" buttons
	glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE); // Make it inVisible, later must call "glfwShowWindow"
	glfwWindowHint(GLFW_FOCUS_ON_SHOW, GLFW_TRUE); // Focus the window once we show it

	// Get Monitor Data
	m_pMonitor = glfwGetPrimaryMonitor();
	m_pVideoMode = glfwGetVideoMode(m_pMonitor);

	// Setup Colors & Refresh rate based on our monitor
	glfwWindowHint(GLFW_RED_BITS, m_pVideoMode->redBits);
	glfwWindowHint(GLFW_GREEN_BITS, m_pVideoMode->greenBits);
	glfwWindowHint(GLFW_BLUE_BITS, m_pVideoMode->blueBits);
	glfwWindowHint(GLFW_REFRESH_RATE, m_pVideoMode->refreshRate);

	// Setup Window Size & Deminsions
	m_iFullScreenWidth = m_pVideoMode->width;
	m_iFullScreenHeight = m_pVideoMode->height;

	// Windowed size is 75% of full screen size!
	m_iWindowedWidth = (m_iFullScreenWidth * 75) / 100;
	m_iWindowedHeight = (m_iFullScreenHeight * 75) / 100;

	// Create Window
	if (GetWindowType() == EWindowMode::WINDOWED_MODE)
	{
		m_iWidth = m_iWindowedWidth;
		m_iHeight = m_iWindowedHeight;

		m_pGLWindow = glfwCreateWindow(m_iWidth, m_iHeight, "Lonely", nullptr, nullptr);

		glfwSetWindowPos(m_pGLWindow, (m_iFullScreenWidth - m_iWidth) / 2, (m_iFullScreenHeight - m_iHeight) / 2);
	}
	else if (GetWindowType() == EWindowMode::FULLSCREEN_MODE)
	{
		m_iWidth = m_iFullScreenWidth;
		m_iHeight = m_iFullScreenHeight;

		m_pGLWindow = glfwCreateWindow(m_iWidth, m_iHeight, "Lonely", m_pMonitor, nullptr);
	}

	if (m_pGLWindow == nullptr)
	{
		syserr("Failed to Initialize GLFW Window");
		return (false);
	}

	// Make window context
	glfwMakeContextCurrent(GetGLWindow());

	// Set Current Window Pointer
	glfwSetWindowUserPointer(GetGLWindow(), this);

	// Initialize GLAD library
	if (gladLoadGLLoader(GLADloadproc(glfwGetProcAddress)) == false)
	{
		syserr("Failed to Initialize GLAD Library");
		return (false);
	}

	// some OpenGL Flags
	glEnable(GL_DEPTH_TEST);
	glDisable(GL_CULL_FACE);
	glFrontFace(GL_CCW); // Treat triangles whose vertices are defined in a counter-clockwise order as the front.
	glCullFace(GL_BACK);
	glViewport(0, 0, m_iWidth, m_iHeight); // Set our viewport to window deminsions

	m_mCursorsPtr[GLFW_ARROW_CURSOR] = glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
	m_mCursorsPtr[GLFW_IBEAM_CURSOR] = glfwCreateStandardCursor(GLFW_IBEAM_CURSOR);
	m_mCursorsPtr[GLFW_CROSSHAIR_CURSOR] = glfwCreateStandardCursor(GLFW_CROSSHAIR_CURSOR);
	m_mCursorsPtr[GLFW_HAND_CURSOR] = glfwCreateStandardCursor(GLFW_HAND_CURSOR);
	m_mCursorsPtr[GLFW_HRESIZE_CURSOR] = glfwCreateStandardCursor(GLFW_HRESIZE_CURSOR);
	m_mCursorsPtr[GLFW_VRESIZE_CURSOR] = glfwCreateStandardCursor(GLFW_VRESIZE_CURSOR);

	// Functions Callbacks
	glfwSetFramebufferSizeCallback(GetGLWindow(), framebuffer_size_callback);
	glfwSetCursorPosCallback(GetGLWindow(), mouse_callback);
	glfwSetScrollCallback(GetGLWindow(), scroll_callback);
	glfwSetKeyCallback(GetGLWindow(), keys_callback);
	glfwSetCursorPos(GetGLWindow(), static_cast<GLdouble>(m_iWidth) / 2, static_cast<GLdouble>(m_iHeight) / 2);
	glfwSetMouseButtonCallback(GetGLWindow(), mouse_button_callback);
	//glDebugMessageCallback(message_callback, nullptr);
	glDebugMessageCallback(MyDebugCallback, nullptr);

	// Show our window
	glfwShowWindow(GetGLWindow());
 	return (true);
}

void CWindow::SetWindowType(GLubyte ubWindowType)
{
	m_ubWindowType = ubWindowType;
}

GLubyte CWindow::GetWindowType() const
{
	return (m_ubWindowType);
}

GLFWwindow* CWindow::GetGLWindow()
{
	return (m_pGLWindow);
}

GLint CWindow::GetWidth() const
{
	return (m_iWidth);
}

GLint CWindow::GetHeight() const
{
	return (m_iHeight);
}

GLfloat CWindow::GetWidthF() const
{
	return static_cast<GLfloat>(m_iWidth);
}

GLfloat CWindow::GetHeightF() const
{
	return static_cast<GLfloat>(m_iHeight);
}

void CWindow::Update()
{
	while (glfwWindowShouldClose(GetGLWindow()) == false)
	{
		glfwPollEvents();

		GLfloat fCurrentFrame = static_cast<GLfloat>(glfwGetTime());
		m_fDeltaTime = fCurrentFrame - m_fLastFrame;
		m_fLastFrame = fCurrentFrame;

		// do some stuff ..
		ProcessInput();

		glfwSwapBuffers(GetGLWindow());
	}
}

void CWindow::ProcessInput()
{
	if (IsKeyDown(GLFW_KEY_ESCAPE))
	{
		glfwSetWindowShouldClose(GetGLWindow(), true);
	}

	if (IsKeyDown(GLFW_KEY_F1))
	{
		if (GetWindowType() != EWindowMode::WINDOWED_MODE)
		{
			SetWindowMode(EWindowMode::WINDOWED_MODE);
		}
	}
	if (IsKeyDown(GLFW_KEY_F1))
	{
		if (GetWindowType() != EWindowMode::FULLSCREEN_MODE)
		{
			SetWindowMode(EWindowMode::FULLSCREEN_MODE);
		}
	}

}

void CWindow::SetCursor(GLint iCursorNum)
{
	m_iCurrentCursor = iCursorNum;
	glfwSetCursor(GetGLWindow(), m_mCursorsPtr[iCursorNum]);
}

void CWindow::SetKeyboardKey(GLint iKey, GLboolean bValue)
{
	if (iKey < 0 || iKey > GLFW_KEY_LAST)
	{
		syserr("Invalid Input, key %d out of range", iKey);
		return;
	}

	m_bKeyBools[iKey] = bValue;
}

void CWindow::SetMouseKey(GLint iKey, GLboolean bValue)
{
	if (iKey < 0 || iKey > GLFW_MOUSE_BUTTON_LAST)
	{
		syserr("Invalid Input, key %d out of range", iKey);
		return;
	}

	m_bMouseKeys[iKey] = bValue;
}

void CWindow::SetMousePosition(GLfloat fX, GLfloat fY)
{
	m_v2MousePos = Vector2D(fX, fY);
}

void CWindow::SetMouseScroll(GLfloat fMouseScrollVal)
{
	m_fMouseScroll = fMouseScrollVal;
	m_bMouseScrollUpdate = true;
}

bool CWindow::IsKeyDown(GLint iKey)
{
	return m_bKeyBools[iKey] == true;
}

bool CWindow::IsKeyUp(GLint iKey)
{
	return m_bKeyBools[iKey] == false;
}

void CWindow::SetWindowMode(const EWindowMode& windowMode)
{
	m_ubWindowType = windowMode;
	if (windowMode == EWindowMode::WINDOWED_MODE)
	{
		m_iWidth = m_iWindowedWidth;
		m_iHeight = m_iWindowedHeight;
		glfwSetWindowMonitor(GetGLWindow(), nullptr, 0, 0, m_iWidth, m_iHeight, m_pVideoMode->refreshRate);
		glfwSetWindowPos(GetGLWindow(), (m_iFullScreenWidth - m_iWidth) / 2, (m_iFullScreenHeight - m_iHeight) / 2);
	}
	else if (windowMode == EWindowMode::FULLSCREEN_MODE)
	{
		m_iWidth = m_iFullScreenWidth;
		m_iHeight = m_iFullScreenHeight;
		glfwSetWindowMonitor(GetGLWindow(), m_pMonitor, 0, 0, m_iWidth, m_iHeight, m_pVideoMode->refreshRate);
		glfwSetWindowPos(GetGLWindow(), 0, 0);
	}
}

void CWindow::framebuffer_size_callback(GLFWwindow* window, GLint width, GLint height)
{
	// Store the raw window pointer.
	CWindow* appWindow = (CWindow*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->ResizeWindow(width, height);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glViewport(0, 0, width, height);
}

void CWindow::mouse_callback(GLFWwindow* window, GLdouble xpos, GLdouble ypos)
{
	// Store the raw window pointer.
	CWindow* appWindow = (CWindow*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->SetMousePosition(static_cast<GLfloat>(xpos), static_cast<GLfloat>(ypos));
}

void CWindow::scroll_callback(GLFWwindow* window, GLdouble xoffset, GLdouble yoffset)
{
	// Store the raw window pointer.
	CWindow* appWindow = (CWindow*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->SetMouseScroll(yoffset);
}

void CWindow::keys_callback(GLFWwindow* window, GLint key, GLint scancode, GLint action, GLint mods)
{
	// Store the raw window pointer.
	CWindow* appWindow = (CWindow*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->SetKeyboardKey(key, action);
}

void CWindow::mouse_button_callback(GLFWwindow* window, GLint button, GLint action, GLint mods)
{
	// Store the raw window pointer.
	CWindow* appWindow = (CWindow*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->SetMouseKey(button, action);
}

void CWindow::ResizeWindow(GLint iWidth, GLint iHeight)
{
	m_iWidth = iWidth;
	m_iHeight = iHeight;
	/*
	Resize FrameBuffers
	*/
}
