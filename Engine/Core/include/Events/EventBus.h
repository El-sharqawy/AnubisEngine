#pragma once

#include "Event.h"
#include "Singleton.h"
#include <functional>
#include <unordered_map>
#include <vector>
#include <utility>

class CEventBus : public CSingleton<CEventBus>
{
public:
    using EventHandler = std::function<void(SIEvent&)>;

    void Subscribe(EEventType type, EventHandler handler)
    {
        m_Listeners[type].push_back(std::move(handler));
    }

    void Fire(SIEvent& event)
    {
        auto it = m_Listeners.find(event.GetType());
        if (it == m_Listeners.end()) return;

        for (auto& handler : it->second)
        {
            if (event.bHandled) break;  // stop if consumed
            handler(event);
        }
    }

    // Fire a temporary event inline
    template<typename T, typename... Args>
    void Emit(Args&&... args)
    {
        T event(std::forward<Args>(args)...);
        Fire(event);
    }

private:
    std::unordered_map<EEventType, std::vector<EventHandler>> m_Listeners;
};