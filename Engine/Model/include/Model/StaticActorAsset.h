#pragma once

#include "API/ActorData.h"
#include "Model/StaticModel.h"

class CStaticActorAsset : public CActorAssetBase
{
public:
    CStaticActorAsset() = default;
    ~CStaticActorAsset() = default;
    EActorAssetType GetType() const override;
    void Clear() override;

    void SetStaticModel(std::shared_ptr<CStaticModel> asset);
    std::shared_ptr<CStaticModel> GetStaticModel() const;
};
