#include "Model/StaticActorAsset.h"
#include "Logging/LogManager.h"

EActorAssetType CStaticActorAsset::GetType() const
{
    return EActorAssetType::ACTOR_ASSET_TYPE_STATIC;
}

void CStaticActorAsset::Clear()
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

void CStaticActorAsset::SetStaticModel(std::shared_ptr<CStaticModel> asset)
{
    m_pModelAsset = std::move(asset);
}

std::shared_ptr<CStaticModel> CStaticActorAsset::GetStaticModel() const
{
    return std::dynamic_pointer_cast<CStaticModel>(m_pModelAsset);
}
