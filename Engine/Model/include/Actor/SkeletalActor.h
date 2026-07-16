#pragma once

#include "API/ActorData.h"
#include "Actor/SkeletalActorAsset.h"
#include "Model/Animator.h"

class CSkeletalActor final : public CActor
{
public:
    void InitializeFromAsset() override;
    void Update(float dt) override;
    void Clear() override;
    bool BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances) override;

    Matrix4 GetWorldMatrix() const;

    std::shared_ptr<CSkeletalActorAsset> GetSkeletalAsset() const
    {
        return std::dynamic_pointer_cast<CSkeletalActorAsset>(m_pAsset);
    }

    std::shared_ptr<CSkeletalModel> GetModel() const
    {
        auto asset = GetSkeletalAsset();
        return asset ? asset->GetSkeletalModel() : nullptr;
    }

    CAnimator* GetAnimator()
    {
        return (m_pAnimator.get());
    }

private:
    std::unique_ptr<CAnimator> m_pAnimator;
    std::vector<SAttachment> m_vAttachments;
};
