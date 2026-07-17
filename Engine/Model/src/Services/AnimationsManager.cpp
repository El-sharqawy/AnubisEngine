#include "Services/AnimationsManager.h"
#include "Logging/LogManager.h"
#include "Services/ActorsManager.h"
#include "Actor/SkeletalActor.h"
#include "Crc/CRC32.h"

void CAnimationsManager::Destroy()
{
	m_mLoadedAnimations.clear();
}

std::shared_ptr<CAnimation> CAnimationsManager::GetAnimationByAssetId(const std::string& stAnimationID)
{
	auto it = m_mLoadedAnimations.find(stAnimationID);
	if (it == m_mLoadedAnimations.end())
	{
		syserr("Animation asset id not found: ", stAnimationID);
		return nullptr;
	}

	SAnimationInfo& info = it->second;

	if (info.pAnimation)
	{
		return info.pAnimation;
	}

	if (!LoadAnimation(stAnimationID))
	{
		syserr("Failed to load animation: ", stAnimationID);
		return nullptr;
	}

	return info.pAnimation;
}

std::shared_ptr<CAnimation> CAnimationsManager::GetAnimation(const std::string& stAnimationID)
{
	// 1. Check if the mesh info exists. If not, the mesh is not defined.
	auto infoIt = m_mLoadedAnimations.find(stAnimationID);
	if (infoIt == m_mLoadedAnimations.end())
	{
		syserr("Animation name '{}' not found.", stAnimationID);
		return nullptr;
	}

	// 2. If the animation object pointer is already set, it's loaded. Return it.
	if (infoIt->second.pAnimation)
	{
		return infoIt->second.pAnimation;
	}

	syserr("Failed to find Animation '{}'.", stAnimationID);
	return nullptr;
}

bool CAnimationsManager::LoadAnimation(const std::string& stAnimationID)
{
	auto it = m_mLoadedAnimations.find(stAnimationID);
	if (it == m_mLoadedAnimations.end())
	{
		return false;
	}

	SAnimationInfo& info = it->second;
	if (info.pAnimation)
	{
		return true;
	}

	auto& actorsMgr = CServiceLocator::Get<CActorsManager>();
	auto actorInfo = actorsMgr.GetActorInfo(info.stActorName);

	auto pActorAsset = actorInfo.pActor->GetAsset();

	if (!pActorAsset)
	{
		return false;
	}

	auto pActorSkeletal = std::dynamic_pointer_cast<CSkeletalActorAsset>(pActorAsset);
	if (!pActorSkeletal)
	{
		return false;
	}

	auto pSkeletalModel = pActorSkeletal->GetSkeletalModel();
	if (!pSkeletalModel)
	{
		return false;
	}

	if (!pSkeletalModel->GetSkeleton())
	{
		return false;
	}

	info.pAnimation = std::make_shared<CAnimation>();
	if (info.pAnimation->LoadFromFile(info.stFilePath, pActorSkeletal->GetSkeletalModel()->GetSkeleton()) == false)
	{
		syserr("Failed to Load Animation {}", info.stFilePath);
		return (false);
	}

	info.eLoadState = info.pAnimation ? ELoadState::LOAD_STATE_LOADED : ELoadState::LOAD_STATE_FAILED;

	info.pAnimation->SetAnimationMode(info.eAnimationMode);
	info.pAnimation->SetAnimationType(info.eAnimatinoType);

	return info.pAnimation != nullptr;
}

bool CAnimationsManager::AddAnimationToJson(const std::string& stAnimationFilePath, const std::string stAnimationID, const SAnimationInfo& info)
{
	nlohmann::json jsonData;
	std::ifstream inputFile(stAnimationFilePath);

	// Check if the file exists and can be opened
	if (inputFile.is_open())
	{
		// Parse the existing JSON data
		inputFile >> jsonData;
		inputFile.close();
	}

	// Add or overwrite the new mesh entry
	jsonData[stAnimationID]["animationID"] = info.stAnimationID;
	jsonData[stAnimationID]["animationPath"] = info.stFilePath;
	jsonData[stAnimationID]["animationName"] = info.stAnimationName;
	jsonData[stAnimationID]["skeletonName"] = info.stActorName; // the actor Skeleton Name
	jsonData[stAnimationID]["animationMode"] = info.eAnimationMode;
	jsonData[stAnimationID]["animationType"] = info.eAnimatinoType;
	jsonData[stAnimationID]["crc32"] = GetCaseCRC32(info.stFilePath);

	// Save the modified JSON back to the file
	std::ofstream outputFile(stAnimationFilePath);
	if (!outputFile.is_open())
	{
		syserr("Failed to open JSON file for writing: {}", stAnimationFilePath);
		return (false);
	}

	// Write the JSON data to the file with pretty-printing
	outputFile << jsonData.dump(4); // Use dump(4) for pretty-printing with 4 spaces
	outputFile.close();

	syslog("Added or updated Animation '{}' in JSON file.", stAnimationID);
	return (true);
}

bool CAnimationsManager::RemoveAnimationFromJson(const std::string& stAnimationFilePath, const std::string stAnimationID)
{
	nlohmann::json jsonData;
	std::ifstream inputFile(stAnimationFilePath);

	// Check if the file exists and can be opened
	if (inputFile.is_open())
	{
		// Parse the existing JSON data
		inputFile >> jsonData;
		inputFile.close();
	}

	// Add or overwrite the new mesh entry
	jsonData.erase(stAnimationID); //["filePath"] = stMeshFilePath;

	// Save the modified JSON back to the file
	std::ofstream outputFile(stAnimationFilePath);
	if (!outputFile.is_open())
	{
		syserr("Failed to open JSON file for writing: {}", stAnimationFilePath);
		return (false);
	}

	// Write the JSON data to the file with pretty-printing
	outputFile << jsonData.dump(4); // Use dump(4) for pretty-printing with 4 spaces
	outputFile.close();

	syslog("Deleted animation '{}' in JSON file '{}'.", stAnimationID, stAnimationFilePath);
	return (true);
}

bool CAnimationsManager::LoadAnimationsFromJsonFile(const std::string& stAnimationFilePath)
{
	std::ifstream inputFile(stAnimationFilePath);
	nlohmann::json jsonData;
	if (inputFile.is_open())
	{
		// File exists, so we read its content.
		inputFile >> jsonData;
		inputFile.close();
	}
	else
	{
		// File doesn't exist. Create a new, empty file.
		std::ofstream outputFile(stAnimationFilePath);
		if (!outputFile.is_open())
		{
			syserr("Failed to create new JSON file at: {}", stAnimationFilePath);
			return (false);
		}

		outputFile << "{}"; // Write an empty JSON object to the file
		outputFile.close();

		syslog("JSON file not found. Created a new empty file at: {}", stAnimationFilePath);
		// Since the file is empty, there are no meshes to load.
		return (true);
	}


	// Iterate through all entries in the JSON object
	for (auto const& [stKey, animationData] : jsonData.items())
	{
		try
		{
			SAnimationInfo info;
			info.stAnimationID = stKey;
			info.stFilePath = animationData.at("animationPath").get<std::string>();
			info.stAnimationName = animationData.at("animationName").get<std::string>();
			info.stActorName = animationData.at("skeletonName").get<std::string>();
			info.eAnimationMode = static_cast<EAnimationMode>(animationData.at("animationMode").get<uint32_t>());
			info.eAnimatinoType = static_cast<EAnimationsTypes>(animationData.at("animationType").get<uint32_t>());
			info.uiCRC32 = animationData.at("crc32").get<uint32_t>();
			info.pAnimation = nullptr;

			m_mLoadedAnimations[info.stAnimationID] = info;

			// This is where you would call GetMesh(meshName) to pre-load all meshes.
			// If you prefer lazy loading, you can skip this step.
			// Call your existing function to load the mesh
			if (!LoadAnimation(info.stAnimationID))
			{
				syserr("Failed to Load Animation: '{}'", info.stAnimationName);
				return (false);
			}

#if defined(ENABLE_ANIMATIONS_MANAGER_LOGS)
			syslog("Loaded Animation '{}' from path '{}'", info.stAnimationName, info.stFilePath);
#endif
		}
		catch (const nlohmann::json::exception& err)
		{
			syserr("SON parsing error for animation: {}", err.what());
			return (false);
		}
	}

	return (true);
}

const std::unordered_map<std::string, SAnimationInfo>& CAnimationsManager::GetAnimations() const
{
	return (m_mLoadedAnimations);
}
