#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include "API/BindingContext.h"
#include "Transform.h"
#include "BoundingBox.h"
#include "API/RenderObject.h"

struct SVulkanContext;
class CActorAssetBase;
class CActor;

enum class EActorAssetType
{
	ACTOR_ASSET_TYPE_STATIC,
	ACTOR_ASSET_TYPE_SKELETAL,
};

enum class EAttachmentType
{
	ATTACHMENT_TYPE_SKINNED,
	ATTACHMENT_TYPE_SOCKET,
};

struct SSocket
{
	std::string strName;
	std::string strBoneName;
	STransform LocalTransform;
};

struct SAttachment
{
	std::shared_ptr<CActorAssetBase> pAsset;
	std::string strSocketName;
	STransform LocalOffset;
	EAttachmentType eType = EAttachmentType::ATTACHMENT_TYPE_SOCKET;
	bool bVisible = true;
};

enum class ELoadState : int32_t
{
	LOAD_STATE_NONE,
	LOAD_STATE_PENDING,
	LOAD_STATE_LOADED,
	LOAD_STATE_FAILED
};

struct SActorAttachmentInfo
{
	std::string stAssetName;
	std::string stFilePath;
	std::string stSocketName;
	std::shared_ptr<CActorAssetBase> pActorAsset;
	SBoundingBox boundingBox; // Optional bounding box for collision detection
	STransform sLocalOffset;
	ELoadState eLoadState;		// Mesh Loading State
	EAttachmentType uiAttachmentType; // static / skeletal mesh
	uint32_t uiCRC32;
	bool bFlipUVs = false;
	bool bIsEnabled = true;
	// Initialize with default values

	SActorAttachmentInfo()
	{
		stFilePath = "";
		uiCRC32 = 0;
		pActorAsset = nullptr;
		bFlipUVs = false;
		bIsEnabled = true;
		uiAttachmentType = EAttachmentType::ATTACHMENT_TYPE_SKINNED;
		eLoadState = ELoadState::LOAD_STATE_NONE;
	}
};

struct SActorInfo
{
	std::string stFilePath;
	std::string stAnimationProfile;
	uint32_t uiCRC32;
	std::shared_ptr<CActor> pActor = nullptr;
	SBoundingBox boundingBox; // Optional bounding box for collision detection
	bool bFlipUVs;
	ELoadState eLoadState;		// Mesh Loading State
	std::vector<SActorAttachmentInfo> vAttachments;

	// Initialize with default values
	SActorInfo()
	{
		stFilePath = "";
		stAnimationProfile = "";
		uiCRC32 = 0;
		pActor = nullptr;
		bFlipUVs = false;
		eLoadState = ELoadState::LOAD_STATE_NONE;
		vAttachments.clear();
	}
};

enum class ELocomotionState
{
	None,
	Idle,
	Walk,
	Run
};

class CActorAssetBase
{
public:
    virtual ~CActorAssetBase() = default;
    virtual EActorAssetType GetType() const = 0;
	virtual void Clear() = 0;

    const std::string& GetName() const { return m_strName; }
    void SetName(std::string name) { m_strName = std::move(name); }

    const std::string& GetModelPath() const { return m_strModelPath; }
    void SetModelPath(std::string path) { m_strModelPath = std::move(path); }

    const std::shared_ptr<CModelAssetBase>& GetModelAsset() const { return m_pModelAsset; }
    void SetModelAsset(std::shared_ptr<CModelAssetBase> asset) { m_pModelAsset = std::move(asset); }

    const std::vector<SAttachment>& GetDefaultAttachments() const { return m_vDefaultAttachments; }
    void AddDefaultAttachment(const SAttachment& attachment) { m_vDefaultAttachments.push_back(attachment); }

	EAttachmentType GetModelAttachmentType() const { return m_eAttachmentType; }
	void SetModelAttachmentType(EAttachmentType type) { m_eAttachmentType = type; }

protected:
    std::string m_strName;
    std::string m_strModelPath;
    std::shared_ptr<CModelAssetBase> m_pModelAsset;
    std::vector<SAttachment> m_vDefaultAttachments;
	EAttachmentType m_eAttachmentType = EAttachmentType::ATTACHMENT_TYPE_SOCKET;
};

class CActor
{
public:
	CActor() = default;
	virtual ~CActor() = default;
	virtual void Clear() = 0;
	virtual bool BuildRenderItemsNew(std::vector<SRenderInstance>& renderInstances) = 0;

	void SetAsset(std::shared_ptr<CActorAssetBase> asset) { m_pAsset = std::move(asset); }
	const std::shared_ptr<CActorAssetBase>& GetAsset() const { return m_pAsset; }

	const STransform& GetTransform() const { return m_transform; }
	void SetTransform(const STransform& transform) { m_transform = transform; m_bTransformDirty = true; }

	virtual Matrix4 GetWorldMatrix() const = 0;

	bool IsVisible() const { return m_bVisible; }
	void SetVisible(bool visible) { m_bVisible = visible; }

	virtual void InitializeFromAsset() = 0;
	virtual void Update(float dt) = 0;

protected:
	std::shared_ptr<CActorAssetBase> m_pAsset;

	STransform m_transform{};
	mutable Matrix4 m_worldMatrix = Matrix4(1.0f);
	mutable bool m_bTransformDirty = true;

	bool m_bVisible = true;
};
