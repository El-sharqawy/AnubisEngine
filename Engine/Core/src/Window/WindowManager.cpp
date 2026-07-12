#include "Stdafx.h"
#include "Window/WindowManager.h"
#include "Logging/LogManager.h"
#include "TypeVector3.h"
#include "Camera/Camera.h"

bool CWindowManager::Initialize(const EWindowMode& windowMode, const EGraphicsAPI& api)
{
	m_eGraphicsAPI = api;
	m_eWindowMode = windowMode;

	if (glfwInit() == false)
	{
		syserr("Failed to Initialize GLFW");
		return (false);
	}

	// Setup No Graphics API .. later will attah vulkan
	if (api == EGraphicsAPI::API_VULKAN)
	{
		glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);
	}
	else if (api == EGraphicsAPI::API_OPENGL)
	{
		glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_API);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
		glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
		glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	}

	// Enable Multi Sampling, for a smoother drawing
	glfwWindowHint(GLFW_SAMPLES, 4);

	// handling resized windows takes special care
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);

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

	std::string stWindowTitle = "Vulkan Engine";

	// Create Window
	if (GetWindowMode() == EWindowMode::MODE_WINDOWED)
	{
		m_iWidth = m_iWindowedWidth;
		m_iHeight = m_iWindowedHeight;

		m_pGLFWWindow = glfwCreateWindow(m_iWidth, m_iHeight, stWindowTitle.c_str(), nullptr, nullptr);
	}
	else if (GetWindowMode() == EWindowMode::MODE_FULLSCREEN)
	{
		m_iWidth = m_iFullScreenWidth;
		m_iHeight = m_iFullScreenHeight;

		m_pGLFWWindow = glfwCreateWindow(m_iWidth, m_iHeight, stWindowTitle.c_str(), m_pMonitor, nullptr);
	}

	if (m_pGLFWWindow == nullptr)
	{
		syserr("Failed to Initialize GLFW Window");
		return (false);
	}

	if (GetWindowMode() == EWindowMode::MODE_WINDOWED)
	{
		glfwSetWindowPos(m_pGLFWWindow, (m_iFullScreenWidth - m_iWidth) / 2, (m_iFullScreenHeight - m_iHeight) / 2);
	}

	// Make window context
	if (api == EGraphicsAPI::API_OPENGL)
	{
		glfwMakeContextCurrent(GetGLFWWindow());
		// Disable VSync (0 = Disabled, 1 = Enabled)
		glfwSwapInterval(GLFW_FALSE);
	}

	// Set Current Window Pointer
	glfwSetWindowUserPointer(GetGLFWWindow(), this);

	// Create Standard Cursors
	m_mCursorsPtr[GLFW_ARROW_CURSOR] = glfwCreateStandardCursor(GLFW_ARROW_CURSOR);
	m_mCursorsPtr[GLFW_IBEAM_CURSOR] = glfwCreateStandardCursor(GLFW_IBEAM_CURSOR);
	m_mCursorsPtr[GLFW_CROSSHAIR_CURSOR] = glfwCreateStandardCursor(GLFW_CROSSHAIR_CURSOR);
	m_mCursorsPtr[GLFW_HAND_CURSOR] = glfwCreateStandardCursor(GLFW_HAND_CURSOR);
	m_mCursorsPtr[GLFW_HRESIZE_CURSOR] = glfwCreateStandardCursor(GLFW_HRESIZE_CURSOR);
	m_mCursorsPtr[GLFW_VRESIZE_CURSOR] = glfwCreateStandardCursor(GLFW_VRESIZE_CURSOR);

	// Functions Callbacks
	glfwSetFramebufferSizeCallback(GetGLFWWindow(), framebuffer_size_callback);
	glfwSetCursorPosCallback(GetGLFWWindow(), mouse_callback);
	glfwSetScrollCallback(GetGLFWWindow(), scroll_callback);
	glfwSetKeyCallback(GetGLFWWindow(), keys_callback);
	glfwSetCursorPos(GetGLFWWindow(), static_cast<double>(m_iWidth) / 2, static_cast<double>(m_iHeight) / 2);
	glfwSetMouseButtonCallback(GetGLFWWindow(), mouse_button_callback);
	//glDebugMessageCallback(message_callback, nullptr);

	// Show our window
	glfwShowWindow(GetGLFWWindow());

	m_pCamera = new CCamera();
	m_pCamera->Initialize(0, m_iWidth, m_iHeight);
	return (true);
}

EGraphicsAPI CWindowManager::GetGraphicsAPI() const
{
	return (m_eGraphicsAPI);
}

void CWindowManager::Destroy()
{
	delete m_pCamera;

	// 1.1 Destroy GL Window
	if (m_pGLFWWindow)
	{
		glfwDestroyWindow(m_pGLFWWindow);
	}

	// 2. Terminate GLFW
	glfwTerminate();

	// 3. Reset Timer resolution (Windows only)
#if defined(ANUBIS_PLATFORM_WINDOWS)
	timeEndPeriod(1); // End the 1 ms timer resolution
#endif
}

void CWindowManager::DestroyGLCursors()
{
	// Cursor Part
	m_iCurrentCursor = GLFW_ARROW_CURSOR;

	glfwDestroyCursor(m_mCursorsPtr[GLFW_ARROW_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_IBEAM_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_CROSSHAIR_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_HAND_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_HRESIZE_CURSOR]);
	glfwDestroyCursor(m_mCursorsPtr[GLFW_VRESIZE_CURSOR]);
	m_mCursorsPtr.clear();
}

EWindowMode CWindowManager::GetWindowMode() const
{
	return (m_eWindowMode);
}

GLFWwindow* CWindowManager::GetGLFWWindow()
{
	return (m_pGLFWWindow);
}

int32_t CWindowManager::GetWidth() const
{
	return (m_iWidth);
}

int32_t CWindowManager::GetHeight() const
{
	return (m_iHeight);
}

float CWindowManager::GetWidthF() const
{
	return static_cast<float>(m_iWidth);
}

float CWindowManager::GetHeightF() const
{
	return static_cast<float>(m_iHeight);
}

void CWindowManager::Update(float deltaTime)
{
	if (glfwGetKey(GetGLFWWindow(), GLFW_KEY_W))
	{
		m_pCamera->ProcessKeyboard(ECameraDirections::DIRECTION_FORWARD, deltaTime);
	}
	if (glfwGetKey(GetGLFWWindow(), GLFW_KEY_S))
	{
		m_pCamera->ProcessKeyboard(ECameraDirections::DIRECTION_BACKWARD, deltaTime);
	}
	if (glfwGetKey(GetGLFWWindow(), GLFW_KEY_D))
	{
		m_pCamera->ProcessKeyboard(ECameraDirections::DIRECTION_RIGHT, deltaTime);
	}
	if (glfwGetKey(GetGLFWWindow(), GLFW_KEY_A))
	{
		m_pCamera->ProcessKeyboard(ECameraDirections::DIRECTION_LEFT, deltaTime);
	}
}

bool CWindowManager::ShouldClose() const
{
	return glfwWindowShouldClose(m_pGLFWWindow) == GLFW_TRUE;
}

void CWindowManager::Close()
{
	glfwSetWindowShouldClose(m_pGLFWWindow, GLFW_TRUE);
}

/**
 * @brief Handles operating system input events.
 *
 * This function polls for and processes input events from the operating system,
 * such as keyboard and mouse events. It ensures that the application responds
 * to user interactions in a timely manner.
 */
void CWindowManager::HandleOsInput()
{
	glfwPollEvents();
}

/**
 * @brief Swaps the front and back buffers of the window.
 *
 * This function is responsible for presenting the rendered frame to the display
 * by swapping the front and back buffers. It should be called after rendering
 * a frame to ensure that the latest visual output is shown to the user.
 */
void CWindowManager::SwapBuffers()
{
	glfwSwapBuffers(GetGLFWWindow());
}

void CWindowManager::SetCursor(int32_t iCursorNum)
{
	m_iCurrentCursor = iCursorNum;
	glfwSetCursor(GetGLFWWindow(), m_mCursorsPtr[iCursorNum]);
}

void CWindowManager::SetKeyboardKey(int32_t key, int32_t action)
{
	if (key < 0 || key > GLFW_KEY_LAST)
	{
		syserr("Invalid Input, key %d out of range", key);
		return;
	}

	if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
	{
		glfwSetWindowShouldClose(GetGLFWWindow(), true);
	}
}

void CWindowManager::SetMouseKey(int32_t key, int32_t action)
{
	if (key < 0 || key > GLFW_MOUSE_BUTTON_LAST)
	{
		syserr("Invalid Input, key %d out of range", key);
		return;
	}
}

void CWindowManager::SetMousePosition(float fX, float fY)
{
	m_pCamera->ProcessMouse(Vector2D(fX, fY));
}

void CWindowManager::SetMouseScroll(float fMouseScrollValY)
{
	m_pCamera->ProcessMouseScroll(fMouseScrollValY);
}

/**
 * @brief Sets the window mode (windowed or fullscreen).
 *
 * This function allows switching between different window modes,
 * such as windowed mode and fullscreen mode. It adjusts the window
 * properties accordingly to match the selected mode.
 *
 * @param windowMode The desired window mode to set.
 */
void CWindowManager::SetWindowMode(const EWindowMode& windowMode)
{
	m_eWindowMode = windowMode;
	if (windowMode == EWindowMode::MODE_WINDOWED)
	{
		m_iWidth = m_iWindowedWidth;
		m_iHeight = m_iWindowedHeight;
		glfwSetWindowMonitor(GetGLFWWindow(), nullptr, 0, 0, m_iWidth, m_iHeight, m_pVideoMode->refreshRate);
		glfwSetWindowPos(GetGLFWWindow(), (m_iFullScreenWidth - m_iWidth) / 2, (m_iFullScreenHeight - m_iHeight) / 2);
	}
	else if (windowMode == EWindowMode::MODE_FULLSCREEN)
	{
		m_iWidth = m_iFullScreenWidth;
		m_iHeight = m_iFullScreenHeight;
		glfwSetWindowMonitor(GetGLFWWindow(), m_pMonitor, 0, 0, m_iWidth, m_iHeight, m_pVideoMode->refreshRate);
		glfwSetWindowPos(GetGLFWWindow(), 0, 0);
	}

	ResizeWindow(m_iWidth, m_iHeight);
}

void CWindowManager::ResizeWindow(int32_t iWidth, int32_t iHeight)
{
	m_iWidth = iWidth;
	m_iHeight = iHeight;

	/*
	Resize FrameBuffers
	*/
}

void CWindowManager::framebuffer_size_callback(GLFWwindow* window, GLint width, GLint height)
{
	// Store the raw window pointer.
	CWindowManager* appWindow = (CWindowManager*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->ResizeWindow(width, height);
}

void CWindowManager::mouse_callback(GLFWwindow* window, GLdouble xpos, GLdouble ypos)
{
	// Store the raw window pointer.
	CWindowManager* appWindow = (CWindowManager*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->SetMousePosition(static_cast<GLfloat>(xpos), static_cast<GLfloat>(ypos));
}

void CWindowManager::scroll_callback(GLFWwindow* window, GLdouble xoffset, GLdouble yoffset)
{
	// Store the raw window pointer.
	CWindowManager* appWindow = (CWindowManager*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->SetMouseScroll(static_cast<GLfloat>(yoffset));
}

void CWindowManager::keys_callback(GLFWwindow* window, GLint key, GLint scancode, GLint action, GLint mods)
{
	// Store the raw window pointer.
	CWindowManager* appWindow = (CWindowManager*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->SetKeyboardKey(key, action);
}

void CWindowManager::mouse_button_callback(GLFWwindow* window, GLint button, GLint action, GLint mods)
{
	// Store the raw window pointer.
	CWindowManager* appWindow = (CWindowManager*)glfwGetWindowUserPointer(window);
	if (!appWindow)
	{
		return;
	}

	appWindow->SetMouseKey(button, action);
}

