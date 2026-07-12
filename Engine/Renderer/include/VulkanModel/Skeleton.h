#pragma once

#include "MeshData.h"
#include "TypeMatrix4.h"
#include <map>

class CSkeleton
{
	friend class CAssimpModelImporter;

public:
	CSkeleton() = default;
	~CSkeleton() = default;

	const std::map<std::string, SBoneInfo>& GetBoneInfoMap() const { return m_mBoneInfoMap; }
	int32_t GetBoneCount() const { return m_iBoneCounter; }

	Matrix4 GetGlobalInverseTransform() const { return m_matGlobalInverseTransform; }

private:
	// Used for Animation
	Matrix4 m_matGlobalInverseTransform;
	std::map<std::string, SBoneInfo> m_mBoneInfoMap;
	int32_t m_iBoneCounter = 0;
};