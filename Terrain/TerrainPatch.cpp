#include "TerrainPatch.h"
#include "VulkanAPI/VulkanBuffer.h"
#include "VulkanAPI/VulkanDevice.h"

bool CTerrainPatch::Initialize()
{
	m_vecVertices.clear();
	m_vecIndices.clear();
	m_vecWaterVertices.clear();
	m_vecWaterIndices.clear();

	// Patch Attributes
	m_iPatchWidth = ETerrainPatchData::PATCH_XSIZE + 1; // Number of squares in the X direction
	m_iPatchDepth = ETerrainPatchData::PATCH_ZSIZE + 1; // Number of squares in the Z direction

	m_iPatchIndex = 0; // Unique index for this patch

	m_uiIndexCount = PATCH_INDEX_COUNT;
	m_uiIndexOffset = 0;  // firstIndex
	m_uiVertexOffset = 0; // baseVertex

	m_uiWaterIndexCount = PATCH_INDEX_COUNT;
	m_uiWaterIndexOffset = 0; // firstIndex
	m_uiWaterVertexOffset = 0; // baseVertex

	m_bUpdateNeeded = true; // Does this patch need to be updated?
	m_bIsWaterPatch = false; // Is this patch a water patch?

	m_fPatchWaterHeight = 0.0f; // Store the height for this specific water patch

	m_sTransform = {};

	m_v3BoundsMin = Vector3D(FLT_MAX);
	m_v3BoundsMax = Vector3D(FLT_MIN);
	m_fBoundsRadius = 1.0f;

	return true;
}

void CTerrainPatch::Clear()
{
	m_vecVertices.clear();
	m_vecIndices.clear();
	m_vecWaterVertices.clear();
	m_vecWaterIndices.clear();

	// Patch Attributes
	m_iPatchWidth = ETerrainPatchData::PATCH_XSIZE + 1; // Number of squares in the X direction
	m_iPatchDepth = ETerrainPatchData::PATCH_ZSIZE + 1; // Number of squares in the Z direction

	m_iPatchIndex = 0; // Unique index for this patch

	m_uiIndexCount = PATCH_INDEX_COUNT;
	m_uiIndexOffset = 0;  // firstIndex
	m_uiVertexOffset = 0; // baseVertex

	m_uiWaterIndexCount = PATCH_INDEX_COUNT;
	m_uiWaterIndexOffset = 0; // firstIndex
	m_uiWaterVertexOffset = 0; // baseVertex

	m_bUpdateNeeded = true; // Does this patch need to be updated?
	m_bIsWaterPatch = false; // Is this patch a water patch?

	m_fPatchWaterHeight = 0.0f; // Store the height for this specific water patch

	m_sTransform = {};

	m_v3BoundsMin = Vector3D(FLT_MAX);
	m_v3BoundsMax = Vector3D(FLT_MIN);
	m_fBoundsRadius = 1.0f;
}

void CTerrainPatch::InitializeVertices()
{
	const float fTotalWidth = static_cast<float>((m_iPatchWidth - 1) * CELL_SCALE_METER);
	const float fTotalDepth = static_cast<float>((m_iPatchDepth - 1) * CELL_SCALE_METER);

	for (int32_t iZ = 0; iZ < m_iPatchDepth; iZ++)
	{
		for (int32_t iX = 0; iX < m_iPatchWidth; iX++)
		{
			// 1. Calculate coordinates (Now both positive)
			float fX = static_cast<float>(iX * CELL_SCALE_METER);
			float fZ = static_cast<float>(iZ * CELL_SCALE_METER);

			// Optional: Uncomment these to center the patch at (0,0,0)
			// fX -= fTotalWidth / 2.0f;
			// fZ -= fTotalDepth / 2.0f;

			STerrainVertex vertex{};
			vertex.m_v3Position = Vector3D(fX, 0.0f, fZ);

			vertex.m_v2TexCoords = Vector2D(static_cast<float>(iX), static_cast<float>(iZ));

			vertex.m_v3Normals = Vector3D(0.0f, 1.0f, 0.0f);
			vertex.m_v4Color = Vector4D(1.0f);
			m_vecVertices.push_back(vertex);
		}
	}
}

void CTerrainPatch::InitializeIndices()
{
	m_vecIndices.clear();
	m_vecIndices.reserve(PATCH_INDEX_COUNT);

	const int32_t verticesPerRow = PATCH_XSIZE + 1;

	for (int32_t iZ = 0; iZ < m_iPatchDepth - 1; iZ++)
	{
		for (int32_t iX = 0; iX < m_iPatchWidth - 1; iX++)
		{
			int32_t iTopLeft = iZ * m_iPatchWidth + iX;
			int32_t iTopRight = iZ * m_iPatchWidth + (iX + 1);
			int32_t iBottomLeft = (iZ + 1) * m_iPatchWidth + iX;
			int32_t iBottomRight = (iZ + 1) * m_iPatchWidth + (iX + 1);

			// First Triangle
			m_vecIndices.push_back(iTopLeft);
			m_vecIndices.push_back(iBottomLeft);
			m_vecIndices.push_back(iTopRight);

			// Second Triangle
			m_vecIndices.push_back(iTopRight);
			m_vecIndices.push_back(iBottomLeft);
			m_vecIndices.push_back(iBottomRight);
		}
	}
}

std::vector<STerrainVertex>& CTerrainPatch::GetVertices()
{
	return (m_vecVertices);
}

const std::vector<STerrainVertex>& CTerrainPatch::GetVertices() const
{
	return (m_vecVertices);
}

std::vector<GLuint>& CTerrainPatch::GetIndices()
{
	return (m_vecIndices);
}

const std::vector<GLuint>& CTerrainPatch::GetIndices() const
{
	return (m_vecIndices);
}

// Patch Accessors and Mutators
void CTerrainPatch::SetPatchWidth(int32_t iWidth)
{
	m_iPatchWidth = iWidth;
	SetUpdateNeed(true);
}

int32_t CTerrainPatch::GetPatchWidth() const
{
	return m_iPatchWidth;
}

void CTerrainPatch::SetPatchDepth(int32_t iDepth)
{
	m_iPatchDepth = iDepth;
	SetUpdateNeed(true);
}

int32_t CTerrainPatch::GetPatchDepth() const
{
	return m_iPatchDepth;
}

void CTerrainPatch::SetPatchIndex(int32_t iPatchIndex)
{
	m_iPatchIndex = iPatchIndex;
}

int32_t CTerrainPatch::GetPatchIndex() const
{
	return m_iPatchIndex;
}

void CTerrainPatch::SetIndexCount(uint32_t indexCount)
{
	m_uiIndexCount = indexCount;
}

uint32_t CTerrainPatch::GetIndexCount() const
{
	return (m_uiIndexCount);
}

void CTerrainPatch::SetIndexOffset(uint32_t indexOffset)
{
	m_uiIndexOffset = indexOffset;
}

uint32_t CTerrainPatch::GetIndexOffset() const
{
	return (m_uiIndexOffset);
}

void CTerrainPatch::SetVertexOffset(uint32_t vertexOffset)
{
	m_uiVertexOffset = vertexOffset;
}

uint32_t CTerrainPatch::GetVertexOffset() const
{
	return (m_uiVertexOffset);
}

void CTerrainPatch::SetWaterIndexCount(uint32_t indexCount)
{
	m_uiWaterIndexCount = indexCount;
}

uint32_t CTerrainPatch::GetWaterIndexCount() const
{
	return (m_uiWaterIndexCount);
}

void CTerrainPatch::SetWaterIndexOffset(uint32_t indexOffset)
{
	m_uiWaterIndexOffset = indexOffset;
}

uint32_t CTerrainPatch::GetWaterIndexOffset() const
{
	return (m_uiWaterIndexOffset);
}

void CTerrainPatch::SetWaterVertexOffset(uint32_t vertexOffset)
{
	m_uiWaterVertexOffset = vertexOffset;
}

uint32_t CTerrainPatch::GetWaterVertexOffset() const
{
	return (m_uiWaterVertexOffset);
}

void CTerrainPatch::SetUpdateNeed(bool bNeedsUpdate)
{
	m_bUpdateNeeded = bNeedsUpdate;
}
bool CTerrainPatch::IsUpdateNeeded() const
{
	return (m_bUpdateNeeded);
}

void CTerrainPatch::SetIsWaterPatch(bool bIsWaterPatch)
{
	m_bIsWaterPatch = bIsWaterPatch;
}

bool CTerrainPatch::IsWaterPatch() const
{
	return (m_bIsWaterPatch);
}

void CTerrainPatch::SetPatchWaterHeight(float fHeight)
{
	m_fPatchWaterHeight = fHeight;
}

float CTerrainPatch::GetPatchWaterHeight() const
{
	return (m_fPatchWaterHeight);
}

STransform& CTerrainPatch::GetTransform()
{
	return (m_sTransform);
}

void CTerrainPatch::SetTransform(const STransform& sTransform)
{
	m_sTransform = sTransform;
}

void CTerrainPatch::SetPosition(const Vector3D& v3Position)
{
	m_sTransform.SetPosition(v3Position);
}

Vector3D CTerrainPatch::GetPosition() const
{
	return (m_sTransform.m_v3Position);
}

Matrix4 CTerrainPatch::GetMatrix() const
{
	return (m_sTransform.GetMatrix());
}

Vector3D& CTerrainPatch::GetVertex(int32_t x, int32_t z)
{
	// 1. Boundary Guard
	// If we ask for something outside the grid, clamp it to the edge
	if (x < 0)
	{
		x = 0;
	}
	if (z < 0)
	{
		z = 0;
	}

	// Clamp to max size
	if (x >= m_iPatchWidth)
	{
		x = m_iPatchWidth - 1;
	}
	if (z >= m_iPatchDepth)
	{
		z = m_iPatchDepth - 1;
	}

	// 2. 2D to 1D Mapping
	// Since we pushed vertices in a loop (for z { for x }), the index is:
	int32_t index = z * m_iPatchWidth + x;

	return m_vecVertices[index].m_v3Position;
}

const Vector3D& CTerrainPatch::GetVertex(int32_t x, int32_t z) const
{
	// 1. Boundary Guard
	// If we ask for something outside the grid, clamp it to the edge
	if (x < 0)
	{
		x = 0;
	}
	if (z < 0)
	{
		z = 0;
	}

	// Clamp to max size
	if (x >= m_iPatchWidth)
	{
		x = m_iPatchWidth - 1;
	}
	if (z >= m_iPatchDepth)
	{
		z = m_iPatchDepth - 1;
	}

	// 2. 2D to 1D Mapping
	// Since we pushed vertices in a loop (for z { for x }), the index is:
	int32_t index = z * m_iPatchWidth + x;

	return m_vecVertices[index].m_v3Position;
}

std::vector<STerrainVertex>& CTerrainPatch::GetPatchWaterVertices()
{
	return (m_vecWaterVertices);
}

const std::vector<STerrainVertex>& CTerrainPatch::GetPatchWaterVertices() const
{
	return (m_vecWaterVertices);
}

std::vector<uint32_t>& CTerrainPatch::GetPatchWaterIndices()
{
	return (m_vecWaterIndices);
}

const std::vector<uint32_t>& CTerrainPatch::GetPatchWaterIndices() const
{
	return (m_vecWaterIndices);
}

bool CTerrainPatch::IsVisible(const SFrustumCulling& frustumCulling) const
{
	return frustumCulling.IsAABBInsideViewFrustum(m_v3BoundsMin, m_v3BoundsMax);
}

void CTerrainPatch::SetBoundsFromAABB(const Vector3D& vMin, const Vector3D& vMax)
{
	m_v3BoundsMin = vMin;
	m_v3BoundsMax = vMax;
}

void CTerrainPatch::SetBoundsRadius(const float fBoundsRadius)
{
	m_fBoundsRadius = fBoundsRadius;
}

Vector3D CTerrainPatch::GetBoundsMin() const
{
	return (m_v3BoundsMin);
}

Vector3D CTerrainPatch::GetBoundsMax() const
{
	return (m_v3BoundsMax);
}

float CTerrainPatch::GetBoundsRadius() const
{
	return (m_fBoundsRadius);
}
