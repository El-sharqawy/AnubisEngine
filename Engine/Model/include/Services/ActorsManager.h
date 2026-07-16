#pragma once

#pragma once

#include <mutex>

#include "ServiceLocator.h"
#include "Singleton.h"
#include "API/ActorData.h"
#include "BoundingBox.h"
#include "TypeMatrix4.h"

#define ENABLE_ACTORS_MANAGER_LOGS

class CActorsManager
{
public:
	CActorsManager() = default;
	~CActorsManager() = default;

	void Destroy();

	// Gets a pointer to an actor.
	// It loads the actor from the file path if it's not already in memory.
	std::shared_ptr<CActorAssetBase> GetActorAsset(const std::string& stActorName);
	std::shared_ptr<CActor> GetActor(const std::string& stActorName);
	SActorInfo GetActorInfo(const std::string& stActorName);
	bool LoadActor(const std::string& stActorName);
	bool LoadActorAttachments(const std::string& stActorName);
	bool LoadActorAnimations(const std::string& stActorName);

	// Json File
	bool AddActorToJson(const std::string& stActorsFilePath, const std::string stActorName, const SActorInfo& info);
	bool RemoveMeshFromJson(const std::string& stActorsFilePath, const std::string stActorName);
	bool LoadMeshesFromJson(const std::string& stActorsFilePath);

	void Update(float deltaTime);
private:
	std::unordered_map<std::string, SActorInfo> m_mLoadedActors = {}; // Maps mesh names to their file paths and CRC32 hashes and physics objects
	std::unordered_map<std::string, SActorAttachmentInfo> m_mLoadedActorsAssets = {}; // Maps assets names to their file paths and CRC32 hashes and physics objects

	// A mutex to protect the global vectors and their sizes during asynchronous loading
	std::mutex m_mtxActorsLoading = {};
};