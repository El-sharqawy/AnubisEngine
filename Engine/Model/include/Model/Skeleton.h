#pragma once

#include "Model/SkeletalMeshData.h"
#include "Model/Bone.h"
#include "TypeMatrix4.h"
#include <map>
#include "Logging/LogManager.h"

class CSkeleton
{
public:
	CSkeleton() = default;
	~CSkeleton() = default;

	std::map<std::string, SBoneInfo>& GetBoneInfoMap() { return m_mBoneInfoMap; }
	const std::map<std::string, SBoneInfo>& GetBoneInfoMap() const { return m_mBoneInfoMap; }
	int32_t GetBoneCount() const { return m_iBoneCounter; }
	void SetBoneCount(int32_t iBoneCount) { m_iBoneCounter = iBoneCount; }
	Matrix4 GetGlobalInverseTransform() const { return m_matGlobalInverseTransform; }
	void SetGlobalInverseTransformMatrix(Matrix4 matrix) { m_matGlobalInverseTransform = matrix; }

private:
	// Used for Animation
	Matrix4 m_matGlobalInverseTransform;
	std::map<std::string, SBoneInfo> m_mBoneInfoMap;
	int32_t m_iBoneCounter = 0;
};