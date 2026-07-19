#include "Core/Scene.h"
#include "API/Entity.h"
#include "Entity/SkeletalMeshComponent.h"

void CScene::ClearEntities()
{
    for (const auto& pEntity : GetEntities())
    {
        if (!pEntity)
            continue;

        if (pEntity->HasComponent<CSkeletalMeshComponent>())
        {
            auto* pActor = pEntity->GetComponent<CSkeletalMeshComponent>();
            if (pActor)
            {
                pActor->Clear();
            }
        }
    }
}

CEntity& CScene::CreateEntity(const std::string& Name)
{
    m_vEntities.emplace_back(std::make_unique<CEntity>(m_uiNextEntityID++, Name));
    return *m_vEntities.back();
}

void CScene::DestroyEntity(EntityID ID)
{
    auto It = std::remove_if(m_vEntities.begin(), m_vEntities.end(), [ID](const std::unique_ptr<CEntity>& pEntity)
        {
            return pEntity && pEntity->GetID() == ID;
        });

    m_vEntities.erase(It, m_vEntities.end());
}

std::vector<std::unique_ptr<CEntity>>& CScene::GetEntities()
{
    return m_vEntities;
}

const std::vector<std::unique_ptr<CEntity>>& CScene::GetEntities() const
{
    return m_vEntities;
}

CEntity* CScene::FindEntityByID(EntityID ID)
{
    for (auto& pEntity : m_vEntities)
    {
        if (pEntity->GetID() == ID)
        {
            return pEntity.get();
        }
    }

    return nullptr;
}

const CEntity* CScene::FindEntityByID(EntityID ID) const
{
    for (const auto& pEntity : m_vEntities)
    {
        if (pEntity->GetID() == ID)
        {
            return pEntity.get();
        }
    }

    return nullptr;
}
