#include "Entity/StaticMeshComponent.h"
#include "Logging/LogManager.h"
#include "Utils/AnubisAssert.h"

bool CStaticMeshComponent::Initialize(CEntity* pOwnerEntity)
{
    ANUBIS_ASSERT(pOwnerEntity != nullptr);
    m_pOwnerEntity = pOwnerEntity;

    if (!m_pAsset || !m_pAsset->GetStaticModel())
    {
        syserr("No Static Asset Given failed ot Initialize");
        return (false);
    }

    return (true);
}

void CStaticMeshComponent::Clear()
{
    if (m_pAsset)
    {
        m_pAsset->Clear();
    }

    for (auto& attachment : m_vAttachments)
    {
        if (attachment.pAsset)
        {
            attachment.pAsset->Clear();
        }
    }
}

void CStaticMeshComponent::SetAsset(std::shared_ptr<CStaticActorAsset> pAsset)
{
    m_pAsset = std::move(pAsset);
}

const std::shared_ptr<CStaticActorAsset>& CStaticMeshComponent::GetAsset() const
{
    return (m_pAsset);
}

void CStaticMeshComponent::AttachAsset(const SAttachment& Attachment)
{
    m_vAttachments.push_back(Attachment);
}

const std::vector<SAttachment>& CStaticMeshComponent::GetAttachments() const
{
    return (m_vAttachments);
}

std::vector<SAttachment>& CStaticMeshComponent::GetAttachments()
{
    return (m_vAttachments);
}

void CStaticMeshComponent::Update(float deltaTime)
{
    // do nothing
}

bool CStaticMeshComponent::BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances)
{
    Matrix4 matActor(1.0f);
    if (m_pOwnerEntity->HasComponent<CTransformComponent>())
    {
        auto* pTransform = m_pOwnerEntity->GetComponent<CTransformComponent>();
        matActor = pTransform->GetWorldMatrix();
    }

    // Render the Main Asset
    AppendRenderItem(matActor, renderInstances);

    // Render Default Attachments
    AppendAttachemntsRenderItems(matActor, GetAsset()->GetDefaultAttachments(), renderInstances);

    // Render Additional Attachments
    AppendAttachemntsRenderItems(matActor, GetAttachments(), renderInstances);
    return (true);
}

bool CStaticMeshComponent::AppendRenderItem(const Matrix4& matActor, std::vector<SRenderInstance>& renderInstances) const
{
    std::shared_ptr<CStaticModel> pStaticModel = GetAsset()->GetStaticModel();
    if (!pStaticModel)
    {
        syserr("Asset {} is not a static asset type", pStaticModel->GetMeshName());
        return false;
    }

    const std::vector<SMeshBatch>& vBatches = pStaticModel->GetNewBatches();

    for (auto& batch : vBatches)
    {
        SRenderInstance renderInstanceItem{};
        renderInstanceItem.pBatch = &batch;
        renderInstanceItem.modelMatrix = matActor;
        renderInstanceItem.skinPaletteFirstMatrix = 0; // no data
        renderInstanceItem.skinMatrixCount = 0;   // no data
        renderInstanceItem.flags |= ERenderItemFlags::RENDER_ITEM_NONE; // Staticc
        renderInstanceItem.depth = 0.0f; // reserved for future use
        renderInstanceItem.sortKey = 0; // reserved for future use

        renderInstances.push_back(renderInstanceItem);
    }

    return (true);
}

bool CStaticMeshComponent::AppendAttachemntsRenderItems(const Matrix4& matActor, const std::vector<SAttachment>& attachments, std::vector<SRenderInstance>& renderInstances)
{
    for (const auto& attachment : attachments)
    {
        if (!attachment.pAsset || !attachment.pAsset->GetModelAsset())
        {
            continue;
        }

        Matrix4 matAttachmentWorld = matActor;
        if (attachment.eType == EAttachmentType::ATTACHMENT_TYPE_SOCKET)
        {
            Matrix4 matLocal = EngineMath::ComposeTransform(
                attachment.LocalOffset.m_v3Position,
                EngineMath::Normalize(attachment.LocalOffset.m_qOrientation),
                attachment.LocalOffset.m_v3Scale
            );

            matAttachmentWorld = matActor * matLocal;
        }

        const std::vector<SMeshBatch>& vBatches = attachment.pAsset->GetModelAsset()->GetNewBatches();
        for (auto& batch : vBatches)
        {
            SRenderInstance renderInstanceItem{};
            renderInstanceItem.pBatch = &batch;
            renderInstanceItem.modelMatrix = matAttachmentWorld;
            renderInstanceItem.skinPaletteFirstMatrix = 0;
            renderInstanceItem.skinMatrixCount = 0;
            renderInstanceItem.flags |= attachment.eType == EAttachmentType::ATTACHMENT_TYPE_SKINNED ? ERenderItemFlags::RENDER_ITEM_SKINNED : ERenderItemFlags::RENDER_ITEM_NONE;
            renderInstanceItem.depth = 0.0f; // reserved for future use
            renderInstanceItem.sortKey = 0; // reserved for future use

            renderInstances.push_back(renderInstanceItem);
        }
    }
    return (true);
}