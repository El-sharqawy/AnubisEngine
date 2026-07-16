#include "Actor/SkeletalActor.h"
#include "Logging/LogManager.h"
#include "Device/PipelinesManager.h"
#include "EngineMath.h"
#include "EngineMathMatrix.h"
#include "EngineMathQuaternion.h"
#include "Services/RenderQueue.h"

void CSkeletalActor::InitializeFromAsset()
{
    auto asset = GetSkeletalAsset();
    if (!asset)
    {
        return;
    }

    m_vAttachments.clear();
    m_vAttachments.reserve(asset->GetDefaultAttachments().size());

    for (const auto& attachmentDef : asset->GetDefaultAttachments())
    {
        SAttachment instance{};
        instance.pAsset = attachmentDef.pAsset;
        instance.strSocketName = attachmentDef.strSocketName;
        instance.LocalOffset = attachmentDef.LocalOffset;
        instance.eType = attachmentDef.eType;
        instance.bVisible = attachmentDef.bVisible;
        m_vAttachments.push_back(std::move(instance));
    }

    m_pAnimator = std::make_unique<CAnimator>();
    m_pAnimator->InitializeForSkeleton(*asset->GetSkeletalModel()->GetSkeleton());
}

void CSkeletalActor::Update(float deltaTime)
{
    m_pAnimator->UpdateAnimation(deltaTime);

}

void CSkeletalActor::Clear()
{
    GetAsset()->Clear();
}

bool CSkeletalActor::BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances)
{
    auto baseAsset = GetAsset() ? GetAsset()->GetModelAsset() : nullptr;
    if (!baseAsset)
    {
        syserr("Actor has no model asset assigned");
        return false;
    }
    auto pSkeletalModel = std::dynamic_pointer_cast<CSkeletalModel>(baseAsset);
    if (!pSkeletalModel)
    {
        syserr("Asset {} is not a skeletal asset type", pSkeletalModel->GetMeshName());
        return false;
    }

    auto& renderQueue = CServiceLocator::Get<CRenderQueue>();
    uint32_t paletteIndex = renderQueue.AllocateSkinPalette(m_pAnimator->GetFinalBoneMatrices());

    // Safe to proceed!
    const std::vector<SMeshBatch>& vBatches = pSkeletalModel->GetNewBatches();
    for (auto& batch : vBatches)
    {
        SRenderInstance renderInstanceItem{};
        renderInstanceItem.pBatch = &batch;
        renderInstanceItem.modelMatrix = GetWorldMatrix();
        renderInstanceItem.skinPaletteIndex = paletteIndex;
        renderInstanceItem.flags = RENDER_ITEM_SKINNED;
        renderInstanceItem.depth = 0.0f; // reserved for future use
        renderInstanceItem.sortKey = 0; // reserved for future use

        renderInstances.push_back(renderInstanceItem);
    }

    // attachments
    for (const auto& attachment : GetAsset()->GetDefaultAttachments())
    {
        if (!attachment.pAsset || !attachment.pAsset->GetModelAsset())
        {
            continue;
        }

        Matrix4 matAttachmentWorld = GetWorldMatrix();
        const std::vector<Matrix4>* pBones = nullptr;

        if (attachment.eType == EAttachmentType::ATTACHMENT_TYPE_SKINNED)
        {
            pBones = &m_pAnimator->GetFinalBoneMatrices();
        }
        else if (attachment.eType == EAttachmentType::ATTACHMENT_TYPE_SOCKET)
        {
            static bool bPrintMatBone = false;
            Matrix4 matBone = m_pAnimator->GetBoneModelTransform(attachment.strSocketName);
            if (bPrintMatBone == false)
            {
                Anubis::PrintMatrix4(matBone);
                bPrintMatBone = true;
            }

            Matrix4 matLocal = EngineMath::ComposeTransform(
                attachment.LocalOffset.m_v3Position,
                EngineMath::Normalize(attachment.LocalOffset.m_qOrientation),
                attachment.LocalOffset.m_v3Scale
            );

            matAttachmentWorld = GetWorldMatrix() * matBone * matLocal;
        }

        const std::vector<SMeshBatch>& vBatches = attachment.pAsset->GetModelAsset()->GetNewBatches();
        for (auto& batch : vBatches)
        {
            SRenderInstance renderInstanceItem{};
            renderInstanceItem.pBatch = &batch;
            renderInstanceItem.modelMatrix = matAttachmentWorld;
            renderInstanceItem.skinPaletteIndex = paletteIndex;
            renderInstanceItem.flags = attachment.eType == EAttachmentType::ATTACHMENT_TYPE_SKINNED ? ERenderItemFlags::RENDER_ITEM_SKINNED : ERenderItemFlags::RENDER_ITEM_NONE;
            renderInstanceItem.depth = 0.0f; // reserved for future use
            renderInstanceItem.sortKey = 0; // reserved for future use

            renderInstances.push_back(renderInstanceItem);
        }
    }

    return (true);
}

Matrix4 CSkeletalActor::GetWorldMatrix() const
{
    m_worldMatrix = Matrix4(1.0f);
    m_worldMatrix = EngineMath::Translate(m_worldMatrix, m_transform.m_v3Position);
    SQuaternion qModelFix = EngineMath::FromXRotation(EngineMath::ToRadians(180.0f), false); // Rotate 180 degrees on X Axis
    SQuaternion qFinalRot = EngineMath::Multiply(m_transform.m_qOrientation, qModelFix);
    m_worldMatrix = m_worldMatrix * EngineMath::ToMatrix4(qFinalRot);
    m_worldMatrix = EngineMath::Scale(m_worldMatrix, m_transform.m_v3Scale);
    m_bTransformDirty = false;

    return (m_worldMatrix);
}
