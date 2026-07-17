#include "Model/Animation.h"
#include "Model/Skeleton.h"
#include <assimp/Importer.hpp>		// C++ importer interface
#include <assimp/postprocess.h>		// Post processing flags
#include <assimp/scene.h>			// Output data structure
#include "Logging/LogManager.h"

bool CAnimation::LoadFromFile(const std::string& stAnimationFile, std::shared_ptr<CSkeleton> pAnimationSkeleton)
{
	m_bValid = false;

	if (!pAnimationSkeleton)
	{
		syserr("CAnimation::LoadFromFile - null skeleton passed for '{}'", stAnimationFile);
		return false;
	}

	Assimp::Importer importer;
	const aiScene* scene = importer.ReadFile(stAnimationFile, aiProcess_Triangulate);

	if (!scene || !scene->mRootNode)
	{
		syserr("CAnimation::LoadFromFile - failed to load '{}': {}", stAnimationFile, importer.GetErrorString());
		return false;
	}

	if (scene->mNumAnimations == 0)
	{
		syserr("CAnimation::LoadFromFile - '{}' contains no animation channels", stAnimationFile);
		return false;
	}

	const aiAnimation* animation = scene->mAnimations[0];
	if (animation->mNumChannels == 0)
	{
		syserr("CAnimation::LoadFromFile - '{}' animation has no bone channels", stAnimationFile);
		return false;
	}

	m_fDuration = static_cast<float>(animation->mDuration);
	m_iTicksPerSecond = static_cast<int32_t>(animation->mTicksPerSecond);

	ReadHeirarchyData(m_sRootNode, scene->mRootNode);
	ReadMissingBones(animation, pAnimationSkeleton);

	m_bValid = true;
	return true;
}

CBone* CAnimation::FindBone(const std::string& name)
{
	auto it = m_mBoneTrackIndex.find(name);
	if (it == m_mBoneTrackIndex.end())
		return nullptr;

	return &m_vBones[it->second];
}

void CAnimation::ReadMissingBones(const aiAnimation* animation, std::shared_ptr<CSkeleton> pAnimationSkeleton)
{
	int32_t size = animation->mNumChannels;

	const auto& skeletonBoneMap = pAnimationSkeleton->GetBoneInfoMap();

	//reading channels(bones engaged in an animation and their keyframes)
	for (int i = 0; i < size; i++)
	{
		auto channel = animation->mChannels[i];
		std::string boneName = channel->mNodeName.data;

		int boneID = -1;

		// Does this animated bone exist in our skeleton?
		auto it = skeletonBoneMap.find(boneName);
		if (it != skeletonBoneMap.end())
		{
			// Yes! It's a real deforming bone. Map it.
			boneID = it->second.iBoneID;
		}
		else
		{
			// No! It's a dummy bone/IK target in the animation file.
			// We still load the track so children can inherit the transform,
			// but we give it ID = -1 so it doesn't try to write to a shader matrix.
			boneID = -1;
		}

        m_vBones.push_back(CBone(boneName, boneID, channel));
		m_mBoneTrackIndex.emplace(boneName, static_cast<int32_t>(m_vBones.size() - 1));
	}

	m_mBoneInfoMap = skeletonBoneMap;
}

void CAnimation::ReadHeirarchyData(SAssimpNodeData& dest, const aiNode* src)
{
	assert(src);

	dest.stName = src->mName.data;
	dest.matTransformation = Anubis::AssimpToMatrix4(src->mTransformation);
	dest.iChildrenCount = src->mNumChildren;

	for (int i = 0; i < src->mNumChildren; i++)
	{
		SAssimpNodeData newData;
		ReadHeirarchyData(newData, src->mChildren[i]);
		dest.vChildren.push_back(newData);
	}
}

int32_t CAnimation::FindBoneTrackIndex(const std::string& name) const
{
	auto it = m_mBoneTrackIndex.find(name);
	if (it == m_mBoneTrackIndex.end())
		return -1;

	return it->second;
}

CBone& CAnimation::GetBoneByIndex(int32_t index)
{
	return m_vBones[index];
}

const CBone& CAnimation::GetBoneByIndex(int32_t index) const
{
	return m_vBones[index];
}

EAnimationMode CAnimation::GetAnimatinoMode() const
{
	return (m_eAnimationMode);
}
void CAnimation::SetAnimationMode(EAnimationMode animMode)
{
	m_eAnimationMode = animMode;
}

EAnimationsTypes CAnimation::GetAnimationType() const
{
	return (m_eAnimationType);
}

void CAnimation::SetAnimationType(EAnimationsTypes animType)
{
	m_eAnimationType = animType;
}
