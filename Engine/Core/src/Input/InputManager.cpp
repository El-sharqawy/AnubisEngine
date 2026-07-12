#include "Stdafx.h"
#include "Input/InputManager.h"
#include "Logging/LogManager.h"

/**
 * @brief Initializes the input manager.
 *
 * Sets up necessary data structures and states for input handling.
 */
void CInputManager::Initialize()
{
	m_aKeyBools.fill(EKeyState::KEY_UP);
	m_aMouseKeys.fill(EKeyState::KEY_UP);

	m_v2MousePos = Vector2D(0.0f, 0.0f);

	m_fMouseScrollVal = 0.0f;;
}

/**
 * @brief Destroys the input manager.
 *
 * Cleans up resources and resets states used by the input manager.
 */
void CInputManager::Destroy()
{
	m_aKeyBools.fill(EKeyState::KEY_UP);
	m_aMouseKeys.fill(EKeyState::KEY_UP);
}

/**
 * @brief Updates the input manager state.
 *
 * This function should be called once per frame to update the state
 * of keys and mouse buttons.
 */
void CInputManager::Update(float deltaTime)
{
	// Update key states (Pressed->Down, Released->Up)
	for (auto& key : m_aKeyBools)
	{
		if (key == EKeyState::KEY_PRESSED)
		{
			key = EKeyState::KEY_DOWN;
		}
		else if (key == EKeyState::KEY_RELEASED)
		{
			key = EKeyState::KEY_UP;
		}
	}

	// Update mouse button states similarly
	for (auto& key : m_aMouseKeys)
	{
		if (key == EKeyState::KEY_PRESSED)
		{
			key = EKeyState::KEY_DOWN;
		}
		else if (key == EKeyState::KEY_RELEASED)
		{
			key = EKeyState::KEY_UP;
		}
	}
}

/**
 * @brief Handles key input events.
 *
 * @param key The key code of the key event.
 * @param pressed True if the key is pressed, false if released.
 */
void CInputManager::OnKey(EInputKey key, bool pressed)
{
	const size_t index = static_cast<size_t>(key);

	if (pressed)
	{
		// m_bKeyBools[key] = (m_bKeyBools[key] == EKeyState::KEY_DOWN) ? EKeyState::KEY_DOWN : EKeyState::KEY_PRESSED;

		// new press vs already down
		m_aKeyBools[index] = (m_aKeyBools[index] == EKeyState::KEY_DOWN || m_aKeyBools[index] == EKeyState::KEY_PRESSED)
			? EKeyState::KEY_DOWN
			: EKeyState::KEY_PRESSED;

	}
	else
	{
		m_aKeyBools[index] = EKeyState::KEY_RELEASED;
	}
}

/**
 * @brief Handles mouse button input events.
 *
 * @param button The mouse button code of the event.
 * @param pressed True if the button is pressed, false if released.
 */
void CInputManager::OnMouseButton(EMouseButton button, bool pressed)
{
	const size_t index = static_cast<size_t>(button);

	if (pressed)
	{
		m_aMouseKeys[index] = (m_aMouseKeys[index] == EKeyState::KEY_DOWN) ? EKeyState::KEY_DOWN : EKeyState::KEY_PRESSED;

		// --- ADDED LOGIC ---
		// Only trigger UI click on the initial press (not while holding)
		if (button == EMouseButton::MOUSE_BUTTON_LEFT && m_aMouseKeys[index] == EKeyState::KEY_PRESSED)
		{
		}
	}
	else
	{
		m_aMouseKeys[index] = EKeyState::KEY_RELEASED;
	}
}

/**
 * @brief Handles mouse movement events.
 *
 * @param v2MousePos The new mouse position.
 */
void CInputManager::OnMouseMove(const Vector2D& v2MousePos)
{
	m_v2MousePos = v2MousePos;
}

/**
 * @brief Handles mouse movement events.
 *
 * @param fMousePosX The new mouse X position.
 * @param fMousePosY The new mouse Y position.
 */
void CInputManager::OnMouseMove(float fMousePosX, float fMousePosY)
{
	m_v2MousePos.x = fMousePosX;
	m_v2MousePos.y = fMousePosY;
}

/**
 * @brief Handles mouse scroll events.
 *
 * @param fScrollOffset The scroll offset value.
 */
void CInputManager::OnMouseScroll(float fScrollOffset)
{
	m_fMouseScrollVal = fScrollOffset;
}

/**
 * brief Checks if a key is currently down.
 *
 * @param iKey The key code to check.
 * @return True if the key is down or was just pressed, false otherwise.
 */
bool CInputManager::IsKeyDown(EInputKey iKey) const
{
	const size_t index = static_cast<size_t>(iKey);

	return (m_aKeyBools[index] == EKeyState::KEY_DOWN) || (m_aKeyBools[index] == EKeyState::KEY_PRESSED);
}

/**
 * brief Checks if a key was just pressed this frame.
 *
 * @param iKey The key code to check.
 * @return True if the key was just pressed, false otherwise.
 */
bool CInputManager::IsKeyPressed(EInputKey iKey) const
{
	const size_t index = static_cast<size_t>(iKey);

	return (m_aKeyBools[index] == EKeyState::KEY_PRESSED);
}

/**
 * brief Checks if a key was just released this frame.
 *
 * @param iKey The key code to check.
 * @return True if the key was just released, false otherwise.
 */
bool CInputManager::IsKeyReleased(EInputKey iKey) const
{
	const size_t index = static_cast<size_t>(iKey);

	return (m_aKeyBools[index] == EKeyState::KEY_RELEASED);
}

/**
 * brief Checks if a mouse button is currently down.
 *
 * @param iKey The mouse button code to check.
 * @return True if the mouse button is down or was just pressed, false otherwise.
 */
bool CInputManager::IsMouseDown(EMouseButton iKey) const
{
	const size_t index = static_cast<size_t>(iKey);

	return (m_aMouseKeys[index] == EKeyState::KEY_DOWN) || (m_aMouseKeys[index] == EKeyState::KEY_PRESSED);
}

/**
 * brief Checks if a mouse button was just pressed this frame.
 *
 * @param iKey The mouse button code to check.
 * @return True if the mouse button was just pressed, false otherwise.
 */
bool CInputManager::IsMousePressed(EMouseButton iKey) const
{
	const size_t index = static_cast<size_t>(iKey);

	return (m_aMouseKeys[index] == EKeyState::KEY_PRESSED);
}

/**
 * brief Checks if a mouse button was just released this frame.
 *
 * @param iKey The mouse button code to check.
 * @return True if the mouse button was just released, false otherwise.
 */
bool CInputManager::IsMouseReleased(EMouseButton iKey) const
{
	const size_t index = static_cast<size_t>(iKey);

	return (m_aMouseKeys[index] == EKeyState::KEY_RELEASED);
}

/**
 * @brief Retrieves the current mouse position.
 *
 * @return The current mouse position as a Vector2D.
 */
Vector2D CInputManager::GetMousePosition() const
{
	return (m_v2MousePos);
}

/**
 * @brief Retrieves the current mouse scroll value.
 *
 * @return The current mouse scroll value as a GLfloat.
 */
float CInputManager::GetMouseScroll() const
{
	return (m_fMouseScrollVal);
}

EInputKey CInputManager::GLFWKeyToAnubisKey(int32_t glfwKey)
{
	switch (glfwKey)
	{
	case GLFW_KEY_0:
		return EInputKey::KEY_0;
	case GLFW_KEY_1:
		return EInputKey::KEY_1;
	case GLFW_KEY_2:
		return EInputKey::KEY_2;
	case GLFW_KEY_3:
		return EInputKey::KEY_3;
	case GLFW_KEY_4:
		return EInputKey::KEY_4;
	case GLFW_KEY_5:
		return EInputKey::KEY_5;
	case GLFW_KEY_6:
		return EInputKey::KEY_6;
	case GLFW_KEY_7:
		return EInputKey::KEY_7;
	case GLFW_KEY_8:
		return EInputKey::KEY_8;
	case GLFW_KEY_9:
		return EInputKey::KEY_9;
	case GLFW_KEY_F1:
		return EInputKey::KEY_F1;
	case GLFW_KEY_F2:
		return EInputKey::KEY_F2;
	case GLFW_KEY_F3:
		return EInputKey::KEY_F3;
	case GLFW_KEY_F4:
		return EInputKey::KEY_F4;
	case GLFW_KEY_F5:
		return EInputKey::KEY_F5;
	case GLFW_KEY_F6:
		return EInputKey::KEY_F6;
	case GLFW_KEY_F7:
		return EInputKey::KEY_F7;
	case GLFW_KEY_F8:
		return EInputKey::KEY_F8;
	case GLFW_KEY_F9:
		return EInputKey::KEY_F9;
	case GLFW_KEY_F10:
		return EInputKey::KEY_F10;
	case GLFW_KEY_F11:
		return EInputKey::KEY_F11;
	case GLFW_KEY_F12:
		return EInputKey::KEY_F12;
	case GLFW_KEY_W:
		return EInputKey::KEY_W;
	case GLFW_KEY_A:
		return EInputKey::KEY_A;
	case GLFW_KEY_S:
		return EInputKey::KEY_S;
	case GLFW_KEY_D:
		return EInputKey::KEY_D;
	case GLFW_KEY_ESCAPE:
		return EInputKey::KEY_ESCAPE;
	case GLFW_KEY_SPACE:
		return EInputKey::KEY_SPACE;
	case GLFW_KEY_LEFT_SHIFT:
		return EInputKey::KEY_LEFT_SHIFT;
	default:
		return EInputKey::KEY_UNKNOWN;
	}
}

EMouseButton CInputManager::GLFWMouseButtonToAnubisKey(int32_t glfwMouseButton)
{
	switch (glfwMouseButton)
	{
	case GLFW_MOUSE_BUTTON_LEFT:
		return EMouseButton::MOUSE_BUTTON_LEFT;
	case GLFW_MOUSE_BUTTON_RIGHT:
		return EMouseButton::MOUSE_BUTTON_RIGHT;
	case GLFW_MOUSE_BUTTON_MIDDLE:
		return EMouseButton::MOUSE_BUTTON_MIDDLE;
	default:
		return EMouseButton::MOUSE_BUTTON_LEFT;
	}
}