#pragma once

#include <cstdint>
#include <string>

enum class EEventCategory : uint32_t
{
    None = 0,
    Window = 1 << 0,
    Input = 1 << 1,
    Keyboard = 1 << 2,
    Mouse = 1 << 3,
    Application = 1 << 4
};

enum class EEventType
{
    None = 0,

    // Window
    WindowResize,
    WindowClose,
    WindowFocus,
    WindowLostFocus,

    // Keyboard
    KeyPressed,
    KeyReleased,
    KeyTyped,

    // Mouse
    MouseButtonPressed,
    MouseButtonReleased,
    MouseMoved,
    MouseScrolled
};

enum class EMouseButton : uint8_t
{
    MOUSE_BUTTON_LEFT = 0,
    MOUSE_BUTTON_RIGHT,
    MOUSE_BUTTON_MIDDLE,
    MOUSE_BUTTON_COUNT
};

enum class EInputKey : uint16_t
{
    KEY_UNKNOWN,
    KEY_1,
    KEY_2,
    KEY_3,
    KEY_4,
    KEY_5,
    KEY_6,
    KEY_7,
    KEY_8,
    KEY_9,
    KEY_0,
    KEY_F1,
    KEY_F2,
    KEY_F3,
    KEY_F4,
    KEY_F5,
    KEY_F6,
    KEY_F7,
    KEY_F8,
    KEY_F9,
    KEY_F10,
    KEY_F11,
    KEY_F12,
    KEY_W,
    KEY_A,
    KEY_S,
    KEY_D,
    KEY_ESCAPE,
    KEY_SPACE,
    KEY_LEFT_SHIFT,
    KEY_MAX_NUM
};

struct SIEvent
{
    virtual ~SIEvent() = default;
    virtual EEventType     GetType()       const = 0;
    virtual EEventCategory GetCategory()   const = 0;
    virtual const char* GetName()       const = 0;

    bool bHandled = false;  // set true to stop propagation
};

// Macro to reduce boilerplate in every event struct
#define EVENT_TYPE(type) \
    static  EEventType    GetStaticType()  { return EEventType::type; } \
    virtual EEventType    GetType()  const override { return GetStaticType(); } \
    virtual const char*   GetName()  const override { return #type; }

#define EVENT_CATEGORY(category) \
    virtual EEventCategory GetCategory() const override { return category; }
