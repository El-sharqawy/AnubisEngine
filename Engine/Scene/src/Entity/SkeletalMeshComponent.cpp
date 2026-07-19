#include "Entity/SkeletalMeshComponent.h"
#include "Logging/LogManager.h"
#include "Utils/AnubisAssert.h"
#include "Services/RenderQueue.h"
#include "Entity/TransformComponent.h"
#include "EngineMath.h"
#include "EngineMathMatrix.h"
#include "EngineMathQuaternion.h"
#include "Model/Animator.h"

bool CSkeletalMeshComponent::Initialize(CEntity* pOwnerEntity)
{
    ANUBIS_ASSERT(pOwnerEntity != nullptr);
    m_pOwnerEntity = pOwnerEntity;

    if (!m_pAsset || !m_pAsset->GetSkeletalModel())
    {
        syserr("No Skeletal Asset Given failed ot Initialize");
        return (false);
    }

    std::shared_ptr<CSkeletalModel> pSkeletalModel = m_pAsset->GetSkeletalModel();

    m_pAnimator = std::make_unique<CAnimator>();
    m_pAnimator->InitializeForSkeleton(*pSkeletalModel->GetSkeleton());
    m_pAnimator->SetAnimationLibrary(m_pAsset->GetAnimations());

    auto& skinPaletteManager = CServiceLocator::Get<CSkinPaletteManager>();
    uint32_t maxBones = static_cast<uint32_t>(m_pAnimator->GetFinalBoneMatrices().size());
    m_uiSkinPaletteHandle = skinPaletteManager.RegisterPalette(maxBones);

    GetAnimator()->PlayAnimation("WarriorMale/OnehandSword/Idle", true);

    return (true);
}

void CSkeletalMeshComponent::Clear()
{
    auto& skinPaletteManager = CServiceLocator::Get<CSkinPaletteManager>();
    skinPaletteManager.ReleasePalette(m_uiSkinPaletteHandle);
    m_uiSkinPaletteHandle = INVALID_SKIN_PALETTE;

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

void CSkeletalMeshComponent::SetAsset(std::shared_ptr<CSkeletalActorAsset> pAsset)
{
    m_pAsset = std::move(pAsset);
}

const std::shared_ptr<CSkeletalActorAsset>& CSkeletalMeshComponent::GetAsset() const
{
	return (m_pAsset);
}

CAnimator* CSkeletalMeshComponent::GetAnimator()
{
    ANUBIS_ASSERT(m_pAnimator != nullptr);
    return m_pAnimator.get();
}

void CSkeletalMeshComponent::AttachAsset(const SAttachment& Attachment)
{
    m_vAttachments.push_back(Attachment);
}

const std::vector<SAttachment>& CSkeletalMeshComponent::GetAttachments() const
{
    return m_vAttachments;
}

std::vector<SAttachment>& CSkeletalMeshComponent::GetAttachments()
{
    return m_vAttachments;
}

void CSkeletalMeshComponent::Update(float deltaTime)
{
    m_pAnimator->UpdateAnimation(deltaTime);
}

bool CSkeletalMeshComponent::BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances)
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

bool CSkeletalMeshComponent::AppendRenderItem(const Matrix4& matActor, std::vector<SRenderInstance>& renderInstances)
{
    auto& skinPaletteManager = CServiceLocator::Get<CSkinPaletteManager>();
    SSkinPaletteGPUView gpuView = skinPaletteManager.UploadPalette(m_uiSkinPaletteHandle, m_pAnimator->GetFinalBoneMatrices());

    std::shared_ptr<CSkeletalModel> pSkeletalModel = GetAsset()->GetSkeletalModel();
    if (!pSkeletalModel)
    {
        syserr("Asset {} is not a skeletal asset type", pSkeletalModel->GetMeshName());
        return false;
    }

    const std::vector<SMeshBatch>& vBatches = pSkeletalModel->GetNewBatches();

    for (auto& batch : vBatches)
    {
        SRenderInstance renderInstanceItem{};
        renderInstanceItem.pBatch = &batch;
        renderInstanceItem.modelMatrix = matActor;
        renderInstanceItem.skinPaletteFirstMatrix = gpuView.firstMatrix;
        renderInstanceItem.skinMatrixCount = gpuView.matrixCount;
        renderInstanceItem.flags |= RENDER_ITEM_SKINNED;
        renderInstanceItem.depth = 0.0f; // reserved for future use
        renderInstanceItem.sortKey = 0; // reserved for future use

        renderInstances.push_back(renderInstanceItem);
    }

    return (true);
}

bool CSkeletalMeshComponent::AppendAttachemntsRenderItems(const Matrix4& matActor, const std::vector<SAttachment>& attachments, std::vector<SRenderInstance>& renderInstances)
{
    auto& skinPaletteManager = CServiceLocator::Get<CSkinPaletteManager>();
    SSkinPaletteGPUView gpuView = skinPaletteManager.UploadPalette(m_uiSkinPaletteHandle, m_pAnimator->GetFinalBoneMatrices());

    for (const auto& attachment : attachments)
    {
        if (!attachment.pAsset || !attachment.pAsset->GetModelAsset())
        {
            continue;
        }

        Matrix4 matAttachmentWorld = matActor;
        if (attachment.eType == EAttachmentType::ATTACHMENT_TYPE_SOCKET)
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

            matAttachmentWorld = matActor * matBone * matLocal;
        }

        const std::vector<SMeshBatch>& vBatches = attachment.pAsset->GetModelAsset()->GetNewBatches();
        for (auto& batch : vBatches)
        {
            SRenderInstance renderInstanceItem{};
            renderInstanceItem.pBatch = &batch;
            renderInstanceItem.modelMatrix = matAttachmentWorld;
            renderInstanceItem.skinPaletteFirstMatrix = gpuView.firstMatrix;
            renderInstanceItem.skinMatrixCount = gpuView.matrixCount;
            renderInstanceItem.flags |= attachment.eType == EAttachmentType::ATTACHMENT_TYPE_SKINNED ? ERenderItemFlags::RENDER_ITEM_SKINNED : ERenderItemFlags::RENDER_ITEM_NONE;
            renderInstanceItem.depth = 0.0f; // reserved for future use
            renderInstanceItem.sortKey = 0; // reserved for future use

            renderInstances.push_back(renderInstanceItem);
        }
    }
    return (true);
}
