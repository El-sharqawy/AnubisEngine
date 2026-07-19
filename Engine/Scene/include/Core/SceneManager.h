#pragma once

#include "Core/Scene.h"
#include "API/ActorData.h"

class CSceneManager
{
public:
    CSceneManager() = default;
    ~CSceneManager() = default;

    void Initialize();
    void Destroy();

    void SpawnActorEntity(const SActorInfo& Info);
    void Update(float dt);
    void Render();

    CScene* GetScene() { return m_pScene; }
    const CScene* GetScene() const { return m_pScene; }

private:
    CScene* m_pScene = nullptr;
};