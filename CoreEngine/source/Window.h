#pragma once

#include <maths.h>
#include <GLFW/glfw3.h>
#include <unordered_map>
#include <array>

enum EWindowMode : GLubyte
{
	WINDOWED_MODE,
	FULLSCREEN_MODE,
};

class CWindow
{
public:
	CWindow();
	~CWindow();
	
	void Clear();
	void Destroy();

	bool InitializeWindow();
	
	// Accessors
	void SetWindowType(GLubyte ubWindowType);
	GLubyte GetWindowType() const;
	GLFWwindow* GetGLWindow();
	GLint GetWidth() const;
	GLint GetHeight() const;
	GLfloat GetWidthF() const;
	GLfloat GetHeightF() const;

	void Update();

	// User Input
	void ProcessInput();
	void SetCursor(GLint iCursorNum);
	void SetKeyboardKey(GLint iKey, GLboolean bValue);
	void SetMouseKey(GLint iKey, GLboolean bValue);
	void SetMousePosition(GLfloat fX, GLfloat fY);
	void SetMouseScroll(GLfloat fMouseScrollVal);

	bool IsKeyDown(GLint iKey);
	bool IsKeyUp(GLint iKey);

	// Set Window Mode
	void SetWindowMode(const EWindowMode& windowMode);

protected:
	// Callbacks
	// glfw: whenever the window size changed (by OS or user resize) this callback function executes
	static void framebuffer_size_callback(GLFWwindow* window, GLint width, GLint height);
	static void mouse_callback(GLFWwindow* window, GLdouble xpos, GLdouble ypos);
	static void scroll_callback(GLFWwindow* window, GLdouble xoffset, GLdouble yoffset);
	static void keys_callback(GLFWwindow* window, GLint key, GLint scancode, GLint action, GLint mods);
	static void mouse_button_callback(GLFWwindow* window, GLint button, GLint action, GLint mods);

	void ResizeWindow(GLint iWidth, GLint iHeight);

private:
	GLFWwindow* m_pGLWindow;
	GLFWmonitor* m_pMonitor;
	const GLFWvidmode* m_pVideoMode;

	GLint m_iWidth;
	GLint m_iHeight;

	GLint m_iFullScreenWidth;
	GLint m_iFullScreenHeight;

	GLint m_iWindowedWidth;
	GLint m_iWindowedHeight;

	GLubyte m_ubWindowType;
	
	// Timing
	GLfloat m_fLastFrame;
	GLfloat m_fDeltaTime;

	// Cursors
	GLint m_iCurrentCursor;
	std::unordered_map<GLint, GLFWcursor*> m_mCursorsPtr;
	Vector2D m_v2MousePos;
	GLfloat m_fMouseScroll;
	GLboolean m_bMouseScrollUpdate;

	// Input
	std::array<GLboolean, 1024> m_bKeyBools;
	std::array<GLboolean, 3> m_bMouseKeys; // 0 -> Left, 1 -> Right, 2 -> Scroll
};