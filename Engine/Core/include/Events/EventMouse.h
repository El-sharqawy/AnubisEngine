#pragma once

#include "Event.h"

struct SIMouseKeyPressedEvent : public SIEvent
{
    EVENT_TYPE(MouseButtonPressed)
    EVENT_CATEGORY(EEventCategory::Mouse)

    EMouseButton iKeyCode = EMouseButton::MOUSE_BUTTON_LEFT;
    bool bIsRepeat = false;

    SIMouseKeyPressedEvent(EMouseButton key, bool isRepeat = false)
        : iKeyCode(key), bIsRepeat(isRepeat) {
    }
};

struct SIMouseKeyReleasedEvent : public SIEvent
{
    EVENT_TYPE(MouseButtonReleased)
    EVENT_CATEGORY(EEventCategory::Mouse)

    EMouseButton iKeyCode = EMouseButton::MOUSE_BUTTON_LEFT;
    explicit SIMouseKeyReleasedEvent(EMouseButton key) : iKeyCode(key) {}
};

struct SIMouseMovedEvent : public SIEvent
{
    EVENT_TYPE(MouseMoved)
    EVENT_CATEGORY(EEventCategory::Mouse)

    float fX = 0.0f;
    float fY = 0.0f;

    SIMouseMovedEvent(float x, float y) : fX(x), fY(y) {}
};

struct SIMouseScrolledEvent : public SIEvent
{
    EVENT_TYPE(MouseScrolled)
    EVENT_CATEGORY(EEventCategory::Mouse)

    float fOffsetY = 0.0f;

    SIMouseScrolledEvent(float oy)
        : fOffsetY(oy) {}
};
