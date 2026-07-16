#include "Actor/StaticActor.h"
#include "Logging/LogManager.h"
#include "Device/PipelinesManager.h"
#include "EngineMath.h"
#include "EngineMathMatrix.h"
#include "EngineMathQuaternion.h"
#include "Services/RenderQueue.h"

void CStaticActor::InitializeFromAsset()
{
    auto asset = GetStaticAsset();
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
}

void CStaticActor::Update(float deltaTime)
{
    if (m_bTransformDirty)
    {
        m_worldMatrix = m_transform.GetMatrix();
        m_bTransformDirty = false;
    }

    for (auto& attachment : m_vAttachments)
    {
        if (!attachment.bVisible || !attachment.pAsset)
            continue;

        Matrix4 parentWorld = m_worldMatrix;

        // If you later support static mesh sockets, resolve socket transform here.
        Matrix4 childWorld = parentWorld * attachment.LocalOffset.GetMatrix();
        // attachment.pAsset->SetTransofrm(STransform(childWorld));
    }
}

void CStaticActor::Clear()
{
    GetAsset()->Clear();
}

bool CStaticActor::BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances)
{
    auto baseAsset = GetAsset() ? GetAsset()->GetModelAsset() : nullptr;
    if (!baseAsset)
    {
        syserr("Actor has no model asset assigned");
        return false;
    }
    auto pStaticModel = std::dynamic_pointer_cast<CStaticModel>(baseAsset);
    if (!pStaticModel)
    {
        syserr("Asset {} is not a static asset type", pStaticModel->GetMeshName());
        return false;
    }

    auto& renderQueue = CServiceLocator::Get<CRenderQueue>();
    uint32_t paletteIndex = UINT32_MAX;

    // Safe to proceed!
    const std::vector<SMeshBatch>& vBatches = pStaticModel->GetNewBatches();
    for (auto& batch : vBatches)
    {
        SRenderInstance renderInstanceItem{};
        renderInstanceItem.pBatch = &batch;
        renderInstanceItem.modelMatrix = GetWorldMatrix();
        renderInstanceItem.skinPaletteIndex = paletteIndex;
        renderInstanceItem.flags = 0;
        renderInstanceItem.depth = 0.0f; // reserved for future use
        renderInstanceItem.sortKey = 0; // reserved for future use

        renderInstances.push_back(renderInstanceItem);
    }
    return (true);
}

Matrix4 CStaticActor::GetWorldMatrix() const
{
    if (m_bTransformDirty)
    {
        m_worldMatrix = Matrix4(1.0f);
        m_worldMatrix = EngineMath::Translate(m_worldMatrix, m_transform.m_v3Position);
        SQuaternion qModelFix = EngineMath::FromXRotation(EngineMath::ToRadians(180.0f), false); // Rotate 180 degrees on X Axis
        SQuaternion qFinalRot = EngineMath::Multiply(m_transform.m_qOrientation, qModelFix);
        m_worldMatrix = m_worldMatrix * EngineMath::ToMatrix4(qFinalRot);
        m_worldMatrix = EngineMath::Scale(m_worldMatrix, m_transform.m_v3Scale);
        m_bTransformDirty = false;
    }

    return (m_worldMatrix);
}
