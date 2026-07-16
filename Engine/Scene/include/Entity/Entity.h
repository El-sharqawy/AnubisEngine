#pragma once

#include <memory>
#include <typeindex>
#include <type_traits>
#include <unordered_map>
#include "Entity/IComponent.h"

class CEntity
{
    using EntityID = uint32_t;

public:
    explicit CEntity(EntityID InID, const std::string& Name = "Entity")
        : m_uiID(InID), m_stName(Name)
    {
    }

    EntityID GetID() const
    {
        return m_uiID;
    }

    CEntity(const CEntity&) = delete;
    CEntity& operator=(const CEntity&) = delete;

    CEntity(CEntity&&) noexcept = default;
    CEntity& operator=(CEntity&&) noexcept = default;

    const std::string& GetName() const { return m_stName; }
    void SetName(const std::string& Name) { m_stName = Name; }

    template<typename T, typename... TArgs>
    T& AddComponent(TArgs&&... Args)
    {
        static_assert(std::is_base_of_v<CIComponent, T>, "T must derive from CIComponent");

        std::type_index Type = std::type_index(typeid(T));
        std::unique_ptr<T> NewComponent = std::make_unique<T>(std::forward<TArgs>(Args)...);
        T* RawPtr = NewComponent.get();

        m_mComponents[Type] = std::move(NewComponent);
        return *RawPtr;
    }

    template<typename T>
    bool HasComponent() const
    {
        static_assert(std::is_base_of_v<CIComponent, T>, "T must derive from CIComponent");

        std::type_index Type = std::type_index(typeid(T));
        return m_mComponents.find(Type) != m_mComponents.end();
    }

    template<typename T>
    T* GetComponent()
    {
        static_assert(std::is_base_of_v<CIComponent, T>, "T must derive from CIComponent");

        std::type_index Type = std::type_index(typeid(T));
        auto It = m_mComponents.find(Type);

        if (It == m_mComponents.end())
            return nullptr;

        return static_cast<T*>(It->second.get());
    }

    template<typename T>
    const T* GetComponent() const
    {
        static_assert(std::is_base_of_v<CIComponent, T>, "T must derive from IComponent");

        std::type_index Type = std::type_index(typeid(T));
        auto It = m_mComponents.find(Type);

        if (It == m_mComponents.end())
            return nullptr;

        return static_cast<const T*>(It->second.get());
    }

    template<typename T>
    void RemoveComponent()
    {
        static_assert(std::is_base_of_v<CIComponent, T>, "T must derive from IComponent");

        std::type_index Type = std::type_index(typeid(T));
        m_mComponents.erase(Type);
    }

private:
    EntityID m_uiID;
    std::string m_stName;
    std::unordered_map<std::type_index, std::unique_ptr<CIComponent>> m_mComponents;
};