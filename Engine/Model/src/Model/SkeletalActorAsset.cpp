#include "Stdafx.h"
#include "Model/SkeletalActorAsset.h"
#include "Model/Animator.h"
#include "Logging/LogManager.h"

EActorAssetType CSkeletalActorAsset::GetType() const
{
    return EActorAssetType::ACTOR_ASSET_TYPE_SKELETAL;
}

void CSkeletalActorAsset::Clear()
{
    if (m_pModelAsset)
    {
        m_pModelAsset->Clear();
    }

    for (auto& attachment : m_vDefaultAttachments)
    {
        if (attachment.pAsset)
        {
            attachment.pAsset->Clear();
        }
    }
}

void CSkeletalActorAsset::SetSkeletalModel(std::shared_ptr<CSkeletalModel> asset)
{
    m_pModelAsset = std::move(asset);
}

std::shared_ptr<CSkeletalModel> CSkeletalActorAsset::GetSkeletalModel() const
{
    if (!m_pModelAsset)
    {
        // Handle the error gracefully (log it, return nullptr, or use a placeholder)
        syslog("No Model Asset Loaded");
        return nullptr;
    }

    return std::dynamic_pointer_cast<CSkeletalModel>(m_pModelAsset);
}

const std::vector<SSocket>& CSkeletalActorAsset::GetSockets() const
{
    return m_vSockets;
}

const SSocket* CSkeletalActorAsset::FindSocket(const std::string& strName) const
{
    for (const auto& socket : m_vSockets)
    {
        if (socket.strName == strName)
            return &socket;
    }

    return nullptr;
}

void CSkeletalActorAsset::AddAnimation(const std::string& stAnimationName, std::shared_ptr<CAnimation> pAnimation)
{
    if (!pAnimation)
    {
        return;
    }

    m_mAnimations[stAnimationName] = std::move(pAnimation);
}

const std::unordered_map<std::string, std::shared_ptr<CAnimation>>& CSkeletalActorAsset::GetAnimations() const
{
    return m_mAnimations;
}
