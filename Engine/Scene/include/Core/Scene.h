#pragma once

#pragma once

#include <memomry>
#include <vector>
#include <string>
#include <cstdint>

class CEntity;

class CScene
{
public:
    using EntityID = uint32_t;

public:
    CScene() = default;
    ~CScene() = default;

    void ClearEntities();

    CEntity& CreateEntity(const std::string& Name = "");
    void DestroyEntity(EntityID ID);

    std::vector<std::unique_ptr<CEntity>>& GetEntities();
    const std::vector<std::unique_ptr<CEntity>>& GetEntities() const;

    CEntity* FindEntityByID(EntityID ID);
    const CEntity* FindEntityByID(EntityID ID) const;

private:
    EntityID m_uiNextEntityID = 1;
    std::vector<std::unique_ptr<CEntity>> m_vEntities;
};