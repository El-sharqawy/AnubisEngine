#pragma once

#include "ServiceLocator.h"
#include "Singleton.h"
#include "Model/Animation.h"
#include "API/ActorData.h"
#include <mutex>
#include <string>

struct SAnimationInfo
{
	std::string stAnimationID; // Unique Key .. crc32 of the path
	std::string stFilePath;
	std::string stAnimationName;
	std::string stActorName;
	EAnimationMode eAnimationMode;
	EAnimationsTypes eAnimatinoType;
	uint32_t uiCRC32;
	ELoadState eLoadState;		// Mesh Loading State
	std::shared_ptr<CAnimation> pAnimation;

	// Initialize with default values
	SAnimationInfo()
	{
		stAnimationID = "";
		stFilePath = "";
		stAnimationName = "";
		stActorName = "";
		eAnimationMode = EAnimationMode::ANIMATION_MODE_GENERAL;
		eAnimatinoType = EAnimationsTypes::ANIMATION_WAIT;
		uiCRC32 = 0;
		eLoadState = ELoadState::LOAD_STATE_NONE;
		pAnimation = nullptr;
	}
};

class CAnimationsManager
{
public:
	CAnimationsManager() = default;
	~CAnimationsManager() = default;

	void Destroy();

	std::shared_ptr<CAnimation> GetAnimationByAssetId(const std::string& stAnimationID);
	std::shared_ptr<CAnimation> GetAnimation(const std::string& stAnimationID);

	bool LoadAnimation(const std::string& stAnimationID);
	
	bool AddAnimationToJson(const std::string& stAnimationFilePath, const std::string stAnimationID, const SAnimationInfo& info);
	bool RemoveAnimationFromJson(const std::string& stAnimationFilePath, const std::string stAnimationID);
	bool LoadAnimationsFromJsonFile(const std::string& stAnimationFilePath);

	const std::unordered_map<std::string, SAnimationInfo>& GetAnimations() const;

private:
	std::unordered_map<std::string, SAnimationInfo> m_mLoadedAnimations = {}; // Maps animations names to their file paths and CRC32 hashes and data

	// A mutex to protect the global vectors and their sizes during asynchronous loading
	std::mutex m_mtxAnimationsLoading = {};

};