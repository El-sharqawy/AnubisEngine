#pragma once

#include "Entity/IComponent.h"
#include "Entity/Entity.h"
#include "Model/StaticActorAsset.h"

class CStaticMeshComponent : public IComponent
{
public:
	CStaticMeshComponent() = default;
	~CStaticMeshComponent() override = default;

	bool Initialize(CEntity* pOwnerEntity);
	void Clear();
	void SetAsset(std::shared_ptr<CStaticActorAsset> pAsset);
	const std::shared_ptr<CStaticActorAsset>& GetAsset() const;
	void AttachAsset(const SAttachment& Attachment);
	const std::vector<SAttachment>& GetAttachments() const;
	std::vector<SAttachment>& GetAttachments();

	void Update(float deltaTime);
	bool BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances);
	bool AppendRenderItem(const Matrix4& matActor, std::vector<SRenderInstance>& renderInstances) const;
	bool AppendAttachemntsRenderItems(const Matrix4& matActor, const std::vector<SAttachment>& attachments, std::vector<SRenderInstance>& renderInstances);

private:
	std::shared_ptr<CStaticActorAsset> m_pAsset;
	std::vector<SAttachment> m_vAttachments; // not default attachments
	CEntity* m_pOwnerEntity;
};