#include "Core/SceneManager.h"
#include "Utils/CoreUtils.h"
#include "Services/ActorsManager.h"
#include "Entity/TransformComponent.h"
#include "Entity/SkeletalMeshComponent.h"
#include "EngineMath.h"
#include "EngineMathQuaternion.h"
#include "Services/RenderQueue.h"
#include "Logging/LogManager.h"
#include "Input/InputManager.h"

void CSceneManager::Initialize()
{
	m_pScene = AnubisNew(CScene, MEM_TAG_ENGINE);

    auto& actorsMgr = CServiceLocator::Get<CActorsManager>();
    const SActorInfo pInfo = actorsMgr.GetActorInfo("Warrior_Male");
    if (pInfo.pActorAsset != nullptr)
    {
        SpawnActorEntity(pInfo);
    }

}

void CSceneManager::Destroy()
{
    if (m_pScene)
    {
        m_pScene->ClearEntities();
    }

    AnubisSafeDelete(m_pScene);
}

void CSceneManager::SpawnActorEntity(const SActorInfo& Info)
{
    CEntity& Entity = m_pScene->CreateEntity("Warrior_Male");
    auto& Transform = Entity.AddComponent<CTransformComponent>();
    Transform.SetPosition(Vector3D(10.0f, 0.0f, 10.0f));
    Transform.SetScale(Vector3D(1.0f, 1.0f, 1.0f));
    SQuaternion qModelFix = EngineMath::FromXRotation(EngineMath::ToRadians(180.0f), false); // Rotate 180 degrees on X Axis
    SQuaternion qFinalRot = EngineMath::Multiply(SQuaternion(1.0f), qModelFix);
    Transform.SetRotation(qFinalRot);
    auto& Actor = Entity.AddComponent<CSkeletalMeshComponent>();

    std::shared_ptr<CSkeletalActorAsset> pActorAsset = std::dynamic_pointer_cast<CSkeletalActorAsset>(Info.pActorAsset);
    Actor.SetAsset(pActorAsset);
    Actor.Initialize(&Entity);
}

void CSceneManager::Update(float deltaTime)
{
    auto& inputMgr = CServiceLocator::Get<CInputManager>();

    for (const auto& pEntity : m_pScene->GetEntities())
    {
        if (!pEntity)
        {
            continue;
        }

        auto* pActorAsset = pEntity->GetComponent<CSkeletalMeshComponent>();

        if (inputMgr.IsKeyDown(EInputKey::KEY_1))
        {
            pActorAsset->GetAnimator()->SetUpdating(false);
        }
        if (inputMgr.IsKeyDown(EInputKey::KEY_2))
        {
            pActorAsset->GetAnimator()->SetUpdating(true);
        }

        pActorAsset->Update(deltaTime);
    }
}

void CSceneManager::Render()
{
    auto& renderQueue = CServiceLocator::Get<CRenderQueue>();

    for (const auto& pEntity : m_pScene->GetEntities())
    {
        if (!pEntity)
        {
            continue;
        }

        auto* pActorAsset = pEntity->GetComponent<CSkeletalMeshComponent>();

        std::vector<SRenderInstance> renderInstances{};
        pActorAsset->BuildRenderItemsNew(renderInstances);
        renderQueue.SubimtRenderItems(renderInstances);
    }
}