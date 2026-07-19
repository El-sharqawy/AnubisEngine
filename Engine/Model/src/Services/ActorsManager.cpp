#include "Services/ActorsManager.h"
#include "Model/SkeletalActorAsset.h"
#include "Services/AssimpModelImporter.h"
#include "Services/AnimationsManager.h"
#include "Crc/CRC32.h"
#include "Logging/LogManager.h"

void CActorsManager::Destroy()
{
	for (auto& actor : m_mLoadedActors)
	{
		actor.second.pActorAsset->Clear();
	}

	m_mLoadedActors.clear();
	m_mLoadedActorsAssets.clear();
}

std::shared_ptr<CActorAssetBase> CActorsManager::GetActorAsset(const std::string& stActorName)
{
	// 1. Check if the mesh info exists. If not, the mesh is not defined.
	auto infoIt = m_mLoadedActors.find(stActorName);
	if (infoIt == m_mLoadedActors.end())
	{
		syserr("Actor name '{}' not found.", stActorName);
		return nullptr;
	}

	// 2. If the mesh object pointer is already set, it's loaded. Return it.
	if (infoIt->second.pActorAsset)
	{
		return infoIt->second.pActorAsset;;
	}

	syserr("Failed to find Actor '{}'.", stActorName);
	return nullptr;
}

SActorInfo CActorsManager::GetActorInfo(const std::string& stActorName)
{
	// 1. Check if the mesh info exists. If not, the mesh is not defined.
	auto infoIt = m_mLoadedActors.find(stActorName);
	if (infoIt == m_mLoadedActors.end())
	{
		syserr("Actor name '{}' not found.", stActorName);
		return {};
	}

	// 2. If the mesh object pointer is already set, it's loaded. Return it.
	return infoIt->second;
}

bool CActorsManager::LoadActor(const std::string& stActorName)
{
	std::unordered_map<std::string, SActorInfo>::iterator actorData = m_mLoadedActors.find(stActorName);
	if (actorData == m_mLoadedActors.end())
	{
		syserr("Actor name '{}' not found.", stActorName);
		return (false);
	}

	if (actorData != m_mLoadedActors.end())
	{
		if (actorData->second.pActorAsset)
		{
			syslog("Actor '{}' is already loaded.", stActorName);
			return (true); // Mesh is already loaded
		}
	}

	const std::string& stMeshPath = actorData->second.stFilePath;
	auto& assimpImporter = CServiceLocator::Get<CAssimpModelImporter>();
	auto pNewActorAsset = assimpImporter.ImportActorAsset(stMeshPath, { actorData->second.bFlipUVs });

	if (!pNewActorAsset)
	{
		syserr("Failed to import actor '{}'.", stActorName);
		return false;
	}

	// Update the map entry with the fully loaded CMesh object and its bounding box
	actorData->second.pActorAsset = pNewActorAsset;
	// actorData->second.boundingBox = pNewActor->GetModelAsset()->ComputeMergedLocalBounds();

	// Load declared attachment assets too.
	LoadActorAttachments(stActorName);

	// Load Animations
	if (actorData->second.pActorAsset->GetType() == EActorAssetType::ACTOR_ASSET_TYPE_SKELETAL)
	{
		if (!LoadActorAnimations(stActorName))
		{
			syserr("Failed to Load Actor {} Animations", stActorName);
		}
	}

#if defined(ENABLE_ACTORS_MANAGER_LOGS)
	syslog("Finalized loading for actor '{}'.", stActorName);
#endif
	return (true);
}

bool CActorsManager::LoadActorAttachments(const std::string& stActorName)
{
	auto it = m_mLoadedActors.find(stActorName);
	if (it == m_mLoadedActors.end())
		return false;

	SActorInfo& actorInfo = it->second;
	for (auto& attachment : actorInfo.vAttachments)
	{
		if (!attachment.bIsEnabled)
			continue;

		if (attachment.stAssetName.empty())
		{
			syslog("Actor '{}' has attachment with empty asset name, skipping.", stActorName);
			continue;
		}

		if (attachment.stFilePath.empty())
		{
			syslog("Attachment '{}' for actor '{}' has empty file path, skipping.", attachment.stAssetName, stActorName);
			continue;
		}

		auto attIt = m_mLoadedActorsAssets.find(attachment.stAssetName);
		if (attIt == m_mLoadedActorsAssets.end())
		{
			syslog("Attachment asset '{}' referenced by '{}' is not registered.", attachment.stAssetName, stActorName);
			continue;
		}

		// Already loaded
		if (attIt->second.pActorAsset)
		{
			attachment.pActorAsset = attIt->second.pActorAsset;
			continue;
		}

		auto& assimpImporter = CServiceLocator::Get<CAssimpModelImporter>();
		auto pActorAsset = assimpImporter.ImportActorAsset(attachment.stFilePath, { attachment.bFlipUVs });

		if (!pActorAsset || !pActorAsset->GetModelAsset())
		{
			syserr("Failed to import actor asset '{}' from '{}'.", attachment.stAssetName, attachment.stFilePath);
			continue;
		}

		// Set Actor Asset Data
		pActorAsset->SetName(attachment.stAssetName);
		pActorAsset->SetModelPath(attachment.stFilePath);
		pActorAsset->SetModelAttachmentType(attachment.uiAttachmentType);

		attIt->second.pActorAsset = pActorAsset;
		attachment.pActorAsset = pActorAsset;

		SAttachment attachedAsset{};
		attachedAsset.pAsset = pActorAsset;
		attachedAsset.strSocketName = attachment.stSocketName;
		attachedAsset.eType = attachment.uiAttachmentType;
		attachedAsset.LocalOffset = attachment.sLocalOffset;
		attachedAsset.bVisible = true;

		it->second.pActorAsset->AddDefaultAttachment(attachedAsset);

		syslog("Loaded attachment asset '{}' for actor '{}'.", attachment.stAssetName, stActorName);
	}

	return (true);
}

bool CActorsManager::LoadActorAnimations(const std::string& stActorName)
{
	auto it = m_mLoadedActors.find(stActorName);
	if (it == m_mLoadedActors.end())
	{
		return false;
	}

	SActorInfo& actorInfo = it->second;
	auto pActorAsset = it->second.pActorAsset;

	if (!pActorAsset)
	{
		return (false); // ??
	}
	std::shared_ptr<CSkeletalActorAsset> pActorSkeletalAsset = std::dynamic_pointer_cast<CSkeletalActorAsset>(pActorAsset);

	if (!pActorSkeletalAsset->GetSkeletalModel())
	{
		syserr("No Model Skeletal Model");
		return (false);
	}

	if (!pActorSkeletalAsset->GetSkeletalModel()->GetSkeleton())
	{
		syserr("No Model Skeleton");
		return false;
	}

	auto& animationsMgr = CServiceLocator::Get<CAnimationsManager>();
	animationsMgr.LoadAnimationsFromJsonFile(actorInfo.stAnimationProfile);

	for (auto& [stAnimID, info] : animationsMgr.GetAnimations())
	{
		if (info.stActorName != stActorName)
		{
			syserr("Faile to Add Animation {}, Different Actor {}", stAnimID, info.stActorName);
			continue;
		}

		if (!animationsMgr.LoadAnimation(stAnimID))
		{
			syserr("Faile to Load Animation {}", stAnimID);
			continue;
		}

		pActorSkeletalAsset->AddAnimation(info.stAnimationID, info.pAnimation);
	}

	return (true);
}

bool CActorsManager::AddActorToJson(const std::string& stActorsFilePath, const std::string stActorName, const SActorInfo& info)
{
	nlohmann::json jsonData;
	std::ifstream inputFile(stActorsFilePath);

	// Check if the file exists and can be opened
	if (inputFile.is_open())
	{
		// Parse the existing JSON data
		inputFile >> jsonData;
		inputFile.close();
	}

	// Add or overwrite the new mesh entry
	jsonData[stActorName]["filePath"] = info.stFilePath;
	jsonData[stActorName]["crc32"] = GetCaseCRC32(info.stFilePath);
	jsonData[stActorName]["flip_uvs"] = info.bFlipUVs;
	jsonData[stActorName]["animationProfile"] = info.stAnimationProfile;

	// Add Attachments Data
	nlohmann::json attachmentsJson = nlohmann::json::array();

	for (const auto& att : info.vAttachments)
	{
		attachmentsJson.push_back({
			{ "name", att.stAssetName },
			{ "asset", att.stFilePath },
			{ "type", att.uiAttachmentType == EAttachmentType::ATTACHMENT_TYPE_SKINNED ? "SkinnedMesh" : "SocketMesh" },
			{ "socket", att.stSocketName },
			{ "enabled", att.bIsEnabled },
			{ "flip_uvs", att.bFlipUVs },
			{ "localOffset", {
				{ "position", { att.sLocalOffset.m_v3Position.x, att.sLocalOffset.m_v3Position.y, att.sLocalOffset.m_v3Position.z } },
				{ "rotation", { att.sLocalOffset.m_qOrientation.w, att.sLocalOffset.m_qOrientation.x, att.sLocalOffset.m_qOrientation.y, att.sLocalOffset.m_qOrientation.z } },
				{ "scale",    { att.sLocalOffset.m_v3Scale.x, att.sLocalOffset.m_v3Scale.y, att.sLocalOffset.m_v3Scale.z } }
			}}
			});
	}

	jsonData[stActorName]["attachments"] = attachmentsJson;

	// Add physics data if the mesh has a physics object
	// TODO Later
	// 
	// Save the modified JSON back to the file
	std::ofstream outputFile(stActorsFilePath);
	if (!outputFile.is_open())
	{
		syserr("Failed to open JSON file for writing: {}", stActorsFilePath);
		return (false);
	}

	// Write the JSON data to the file with pretty-printing
	outputFile << jsonData.dump(2); // Use dump(4) for pretty-printing with 4 spaces
	outputFile.close();

	syslog("Added or updated Actor '{}' in JSON file.", stActorName);
	return (true);
}

bool CActorsManager::RemoveMeshFromJson(const std::string& stActorsFilePath, const std::string stActorName)
{
	nlohmann::json jsonData;
	std::ifstream inputFile(stActorsFilePath);

	// Check if the file exists and can be opened
	if (inputFile.is_open())
	{
		// Parse the existing JSON data
		inputFile >> jsonData;
		inputFile.close();
	}

	// Add or overwrite the new mesh entry
	jsonData.erase(stActorName); //["filePath"] = stMeshFilePath;

	// Save the modified JSON back to the file
	std::ofstream outputFile(stActorsFilePath);
	if (!outputFile.is_open())
	{
		syserr("Failed to open JSON file for writing: {}", stActorsFilePath);
		return (false);
	}

	// Write the JSON data to the file with pretty-printing
	outputFile << jsonData.dump(2); // Use dump(4) for pretty-printing with 4 spaces
	outputFile.close();

	syslog("Deleted actor '{}' in JSON file '{}'.", stActorName, stActorsFilePath);
	return (true);
}

bool CActorsManager::LoadMeshesFromJson(const std::string& stActorsFilePath)
{
	std::ifstream inputFile(stActorsFilePath);
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
		std::ofstream outputFile(stActorsFilePath);
		if (!outputFile.is_open())
		{
			syserr("Failed to create new JSON file at: {}", stActorsFilePath);
			return (false);
		}

		outputFile << "{}"; // Write an empty JSON object to the file
		outputFile.close();

		syslog("JSON file not found. Created a new empty file at: {}", stActorsFilePath);
		// Since the file is empty, there are no meshes to load.
		return (true);
	}

	// Iterate through all entries in the JSON object
	for (auto const& [actorName, actorData] : jsonData.items())
	{
		try
		{
			SActorInfo info;

			info.stFilePath = actorData.at("filePath").get<std::string>();
			info.uiCRC32 = actorData.at("crc32").get<uint32_t>();
			info.bFlipUVs = actorData.value("flip_uvs", true); // Default to true if not specified
			info.stAnimationProfile = actorData.at("animationProfile").get<std::string>();

			// Parse the attachment data
			if (actorData.contains("attachments") && actorData["attachments"].is_array())
			{
				for (const auto& att : actorData["attachments"])
				{
					SActorAttachmentInfo attachmentInfo{};

					attachmentInfo.stAssetName = att.value("name", "");
					attachmentInfo.stFilePath = att.value("asset", "");
					attachmentInfo.stSocketName = att.value("socket", "");
					attachmentInfo.bIsEnabled = att.value("enabled", true);
					attachmentInfo.bFlipUVs = att.value("flip_uvs", false);

					const std::string type = att.value("type", "SocketMesh");
					attachmentInfo.uiAttachmentType = (type == "SkinnedMesh") ? EAttachmentType::ATTACHMENT_TYPE_SKINNED : EAttachmentType::ATTACHMENT_TYPE_SOCKET;

					if (att.contains("localOffset"))
					{
						const auto& off = att["localOffset"];

						if (off.contains("position"))
						{
							attachmentInfo.sLocalOffset.m_v3Position = Vector3D(
								off["position"][0].get<float>(),
								off["position"][1].get<float>(),
								off["position"][2].get<float>());
						}

						if (off.contains("rotation"))
						{
							attachmentInfo.sLocalOffset.m_qOrientation = SQuaternion(
								off["rotation"][0].get<float>(),
								off["rotation"][1].get<float>(),
								off["rotation"][2].get<float>(),
								off["rotation"][3].get<float>());
						}

						if (off.contains("scale"))
						{
							attachmentInfo.sLocalOffset.m_v3Scale = Vector3D(
								off["scale"][0].get<float>(),
								off["scale"][1].get<float>(),
								off["scale"][2].get<float>());
						}
					}

					info.vAttachments.push_back(attachmentInfo);
					m_mLoadedActorsAssets[attachmentInfo.stAssetName] = attachmentInfo;
				}
			}

			info.pActorAsset = nullptr;
			m_mLoadedActors[actorName] = info;

			// This is where you would call GetMesh(meshName) to pre-load all meshes.
			// If you prefer lazy loading, you can skip this step.
			// Call your existing function to load the mesh
			if (!LoadActor(actorName))
			{
				syserr("Failed to Load Actor: '{}'", actorName);
				return (false);
			}

#if defined(ENABLE_ACTORS_MANAGER_LOGS)
			syslog("Loaded Actor '{}' from path '{}'", actorName, info.stFilePath);
#endif
		}
		catch (const nlohmann::json::exception& err)
		{
			syserr("SON parsing error for actor '{}': {}", actorName, err.what());
			return (false);
		}
	}

	return (true);
}
