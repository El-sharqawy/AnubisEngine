#pragma once

#include "CoreEnums.h"
#include <unordered_map>
#include "Singleton.h"

class CCamera;

class CWindowManager : public CSingleton<CWindowManager>
{
public:
	CWindowManager() = default;
	~CWindowManager() = default;

	bool Initialize(const EWindowMode& windowMode, const EGraphicsAPI& api);
	EGraphicsAPI GetGraphicsAPI() const;

	// Vulkan
	bool InitializeVulkan();

	void Destroy();

	void DestroyGLCursors();

	// Accessors
	EWindowMode GetWindowMode() const;
	GLFWwindow* GetGLFWWindow();
	int32_t GetWidth() const;
	int32_t GetHeight() const;
	float GetWidthF() const;
	float GetHeightF() const;
	void Update(float deltaTime);

	bool ShouldClose() const;
	void Close();

	/**
	 * @brief Handles operating system input events.
	 *
	 * This function polls for and processes input events from the operating system,
	 * such as keyboard and mouse events. It ensures that the application responds
	 * to user interactions in a timely manner.
	 */
	void HandleOsInput();

	/**
	 * @brief Swaps the front and back buffers of the window.
	 *
	 * This function is responsible for presenting the rendered frame to the display
	 * by swapping the front and back buffers. It should be called after rendering
	 * a frame to ensure that the latest visual output is shown to the user.
	 */
	void SwapBuffers();

	/**
	 * @brief Clears the color and depth buffers to prepare for a new frame.
	 *
	 * This function sets the clear color to a default value (black with full opacity)
	 * and then clears both the color buffer and the depth buffer using glClear.
	 * This ensures that the previous frame's contents do not interfere with the new frame.
	 */

	 // User Input
	void SetCursor(int32_t iCursorNum);
	void SetKeyboardKey(int32_t key, int32_t action);
	void SetMouseKey(int32_t key, int32_t action);
	void SetMousePosition(float fX, float fY);
	void SetMouseScroll(float fMouseScrollValY);

	void SetWindowMode(const EWindowMode& windowMode);

	CCamera* GetCamera() { return m_pCamera; }
protected:
	void ResizeWindow(int32_t iWidth, int32_t iHeight);

	// Callbacks
	// glfw: whenever the window size changed (by OS or user resize) this callback function executes
	static void framebuffer_size_callback(GLFWwindow* window, GLint width, GLint height);
	static void mouse_callback(GLFWwindow* window, GLdouble xpos, GLdouble ypos);
	static void scroll_callback(GLFWwindow* window, GLdouble xoffset, GLdouble yoffset);
	static void keys_callback(GLFWwindow* window, GLint key, GLint scancode, GLint action, GLint mods);
	static void mouse_button_callback(GLFWwindow* window, GLint button, GLint action, GLint mods);

private:
	EGraphicsAPI m_eGraphicsAPI = EGraphicsAPI::API_NONE;

	GLFWwindow* m_pGLFWWindow = nullptr;
	GLFWmonitor* m_pMonitor = nullptr;
	const GLFWvidmode* m_pVideoMode = nullptr;

	// Window size
	int32_t m_iWidth = 0;
	int32_t m_iHeight = 0;

	int32_t m_iFullScreenWidth = 0;
	int32_t m_iFullScreenHeight = 0;

	int32_t m_iWindowedWidth = 0;
	int32_t m_iWindowedHeight = 0;

	EWindowMode m_eWindowMode = EWindowMode::MODE_WINDOWED;

	bool m_bLimitFPS = false;
	int32_t m_iTargetFPS = 0;

	// Cursors
	int32_t m_iCurrentCursor = GLFW_ARROW_CURSOR;
	std::unordered_map<int32_t, GLFWcursor*> m_mCursorsPtr;

	CCamera* m_pCamera = nullptr;
};