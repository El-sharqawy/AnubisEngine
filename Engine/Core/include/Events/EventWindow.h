#pragma once

#include "Event.h"

struct SIWindowResizeEvent : public SIEvent
{
    EVENT_TYPE(WindowResize)
    EVENT_CATEGORY(EEventCategory::Window)

    int32_t iWidth = 0;
    int32_t iHeight = 0;

    SIWindowResizeEvent(int32_t w, int32_t h)
        : iWidth(w), iHeight(h) {}
};

struct SIWindowCloseEvent : public SIEvent
{
    EVENT_TYPE(WindowClose)
    EVENT_CATEGORY(EEventCategory::Window)
};
