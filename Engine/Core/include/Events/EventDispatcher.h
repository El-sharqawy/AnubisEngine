#pragma once

#include "Event.h"
#include <functional>

class CEventDispatcher
{
public:
    explicit CEventDispatcher(SIEvent& event)
        : m_Event(event) {
    }

    // Calls fn only if the event matches type T
    template<typename T>
    bool Dispatch(std::function<bool(T&)> fn)
    {
        if (m_Event.GetType() == T::GetStaticType())
        {
            m_Event.bHandled = fn(static_cast<T&>(m_Event));
            return true;
        }
        return false;
    }

private:
    SIEvent& m_Event;
};