#pragma once

#include "API/ActorData.h"
#include "API/RenderObject.h"
#include "Model/Animation.h"

#include "Model/SkeletalModel.h"

class CSkeletalActorAsset final : public CActorAssetBase
{
public:
    CSkeletalActorAsset() = default;
    ~CSkeletalActorAsset() override = default;
    EActorAssetType GetType() const override;
    void Clear() override;

    void SetSkeletalModel(std::shared_ptr<CSkeletalModel> asset);
    std::shared_ptr<CSkeletalModel> GetSkeletalModel() const;

    const std::vector<SSocket>& GetSockets() const;
    const SSocket* FindSocket(const std::string& name) const;

    void AddAnimation(const std::string& name, std::shared_ptr<CAnimation> animation);
    const std::unordered_map<std::string, std::shared_ptr<CAnimation>>& GetAnimations() const;

private:
    std::vector<SSocket> m_vSockets;
    std::unordered_map<std::string, std::shared_ptr<CAnimation>> m_mAnimations;
};
