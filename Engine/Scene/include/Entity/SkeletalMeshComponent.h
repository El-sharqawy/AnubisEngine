#pragma once

#include "API/IComponent.h"
#include "API/Entity.h"
#include "Model/SkeletalActorAsset.h"
#include "Model/Animator.h"
#include "Services/SkinPaletteManager.h"

class CSkeletalMeshComponent : public IComponent
{
public:
	CSkeletalMeshComponent() = default;
	~CSkeletalMeshComponent() override = default;

	bool Initialize(CEntity* pOwnerEntity);
	void Clear();
	void SetAsset(std::shared_ptr<CSkeletalActorAsset> pAsset);
	const std::shared_ptr<CSkeletalActorAsset>& GetAsset() const;
	CAnimator* GetAnimator();
	void AttachAsset(const SAttachment& Attachment);
	const std::vector<SAttachment>& GetAttachments() const;
	std::vector<SAttachment>& GetAttachments();

	void Update(float deltaTime);

	bool BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances);
	bool AppendRenderItem(const Matrix4& matActor, std::vector<SRenderInstance>& renderInstances);
	bool AppendAttachemntsRenderItems(const Matrix4& matActor, const std::vector<SAttachment>& attachments, std::vector<SRenderInstance>& renderInstances);

private:
	std::shared_ptr<CSkeletalActorAsset> m_pAsset;
	std::unique_ptr<CAnimator> m_pAnimator;
	std::vector<SAttachment> m_vAttachments; // not default attachments
	SkinPaletteHandle m_uiSkinPaletteHandle = INVALID_SKIN_PALETTE;
};