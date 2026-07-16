#pragma once

#include "API/ActorData.h"
#include "Actor/StaticActorAsset.h"

class CStaticActor final : public CActor
{
public:
    void InitializeFromAsset() override;
    void Update(float deltaTime) override;
    void Clear() override;
    bool BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances) override;
    Matrix4 GetWorldMatrix() const override;

    std::shared_ptr<CStaticActorAsset> GetStaticAsset() const
    {
        return std::dynamic_pointer_cast<CStaticActorAsset>(m_pAsset);
    }

    std::shared_ptr<CStaticModel> GetModel() const
    {
        auto asset = GetStaticAsset();
        return asset ? asset->GetStaticModel() : nullptr;
    }

private:
    std::vector<SAttachment> m_vAttachments;
};
