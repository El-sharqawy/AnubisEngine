#pragma once

#include "Event.h"

struct SIKeyPressedEvent : public SIEvent
{
    EVENT_TYPE(KeyPressed)
    EVENT_CATEGORY(EEventCategory::Keyboard)

    EInputKey iKeyCode = EInputKey::KEY_UNKNOWN;
    bool bIsRepeat = false;

    SIKeyPressedEvent(EInputKey key, bool isRepeat = false)
        : iKeyCode(key), bIsRepeat(isRepeat) {
    }
};

struct SIKeyReleasedEvent : public SIEvent
{
    EVENT_TYPE(KeyReleased)
    EVENT_CATEGORY(EEventCategory::Keyboard)

    EInputKey iKeyCode = EInputKey::KEY_UNKNOWN;
    explicit SIKeyReleasedEvent(EInputKey key) : iKeyCode(key) {}
};
