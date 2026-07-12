#pragma once

#include <vulkan/vulkan.h>
#include <array>
#include <vector>

#include "TypeVector2.h"
#include "TypeVector3.h"
#include "TerrainData.h"
#include "Transform.h"
#include "Frustum.h"

class CVulkanBuffer;
class CVulkanDevice;

class CTerrainPatch
{
public:
	CTerrainPatch() = default;
	~CTerrainPatch() = default;

	bool Initialize();
	void Clear();

	void InitializeVertices();
	void InitializeIndices();

public:
	// Patch Accessors and Mutators
	std::vector<STerrainVertex>& GetVertices();
	const std::vector<STerrainVertex>& GetVertices() const;
	std::vector<uint32_t>& GetIndices();
	const std::vector<uint32_t>& GetIndices() const;

	void SetPatchWidth(int32_t iWidth);
	int32_t GetPatchWidth() const;

	void SetPatchDepth(int32_t iDepth);
	int32_t GetPatchDepth() const;

	void SetPatchIndex(int32_t iPatchIndex);
	int32_t GetPatchIndex() const;

	void SetIndexCount(uint32_t indexCount);
	uint32_t GetIndexCount() const;

	void SetIndexOffset(uint32_t indexOffset);
	uint32_t GetIndexOffset() const;

	void SetVertexOffset(uint32_t vertexOffset);
	uint32_t GetVertexOffset() const;

	void SetWaterIndexCount(uint32_t indexCount);
	uint32_t GetWaterIndexCount() const;

	void SetWaterIndexOffset(uint32_t indexOffset);
	uint32_t GetWaterIndexOffset() const;

	void SetWaterVertexOffset(uint32_t vertexOffset);
	uint32_t GetWaterVertexOffset() const;

	void SetUpdateNeed(bool bNeedsUpdate);
	bool IsUpdateNeeded() const;

	void SetIsWaterPatch(bool bIsWaterPatch);
	bool IsWaterPatch() const;

	void SetPatchWaterHeight(float fHeight);
	float GetPatchWaterHeight() const;

	// Transform Accessor
	STransform& GetTransform();
	void SetTransform(const STransform& sTransform);

	void SetPosition(const Vector3D& v3Position);
	Vector3D GetPosition() const;

	// Get Model Matrix
	Matrix4 GetMatrix() const;

	// Get Vertex at (X,Z) coordinate
	Vector3D& GetVertex(int32_t x, int32_t z);
	const Vector3D& GetVertex(int32_t x, int32_t z) const;

	std::vector<STerrainVertex>& GetPatchWaterVertices();
	const std::vector<STerrainVertex>& GetPatchWaterVertices() const;

	std::vector<uint32_t>& GetPatchWaterIndices();
	const std::vector<uint32_t>& GetPatchWaterIndices() const;

	bool IsVisible(const SFrustumCulling& frustumCulling) const;

	void SetBoundsFromAABB(const Vector3D& vMin, const Vector3D& vMax);
	void SetBoundsRadius(const float fBoundsRadius);

	Vector3D GetBoundsMin() const;
	Vector3D GetBoundsMax() const;
	float GetBoundsRadius() const;

private:
	// patch vertex data
	std::vector<STerrainVertex> m_vecVertices = {};
	std::vector<uint32_t> m_vecIndices = {};

	std::vector<STerrainVertex> m_vecWaterVertices = {};
	std::vector<uint32_t> m_vecWaterIndices = {};

	// patch properties
	int32_t m_iPatchWidth = ETerrainPatchData::PATCH_XSIZE + 1; // Number of squares in the X direction
	int32_t m_iPatchDepth = ETerrainPatchData::PATCH_ZSIZE + 1; // Number of squares in the Z direction

	int32_t m_iPatchIndex = 0; // Unique index for this patch

	// draw parameters
	uint32_t m_uiIndexCount = PATCH_INDEX_COUNT;
	uint32_t m_uiIndexOffset = 0;  // firstIndex
	uint32_t m_uiVertexOffset = 0; // baseVertex

	uint32_t m_uiWaterIndexCount = PATCH_INDEX_COUNT;
	uint32_t m_uiWaterIndexOffset = 0; // firstIndex
	uint32_t m_uiWaterVertexOffset = 0; // baseVertex

	bool m_bUpdateNeeded = true; // Does this patch need to be updated?
	bool m_bIsWaterPatch = false; // Is this patch a water patch?

	float m_fPatchWaterHeight = 0.0f; // Store the height for this specific water patch

	STransform m_sTransform = {}; // Transform for positioning the patch

	Vector3D m_v3BoundsMin = Vector3D(FLT_MAX);
	Vector3D m_v3BoundsMax = Vector3D(FLT_MIN);
	float m_fBoundsRadius = 1.0f;
};