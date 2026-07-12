#include "Terrain.h"
#include <algorithm>
#include "EngineMathVectors.h"

void CTerrain::Initialize()
{
	m_stTerrainName = "AreaTerrain";	// Terrain Name
	m_bReady = false;					// Is Terrain Ready
	m_iTerrainNum = -1;					// Terrain Number
	m_iTerrCoordX = m_iTerrCoordZ = 0;	// Terrain Coordinates

	// Initialize Terrain Patches
	m_vecTerrainPatches.resize(PATCH_XCOUNT * PATCH_ZCOUNT);

	m_pParentMap = nullptr; // Parent Terrain Map
}

void CTerrain::Clear()
{
	m_stTerrainName = "AreaTerrain";	// Terrain Name
	m_bReady = false;					// Is Terrain Ready
	m_iTerrainNum = -1;					// Terrain Number
	m_iTerrCoordX = m_iTerrCoordZ = 0;	// Terrain Coordinates

	// Initialize Terrain Patches
	m_vecTerrainPatches.resize(PATCH_XCOUNT * PATCH_ZCOUNT);

	m_pParentMap = nullptr; // Parent Terrain Map
}

void CTerrain::GenerateTerrainPatches()
{
	for (int32_t iPatchZ = 0; iPatchZ < PATCH_ZCOUNT; iPatchZ++)
	{
		for (int32_t iPatchX = 0; iPatchX < PATCH_XCOUNT; iPatchX++)
		{
			GenerateTerrainPatches(iPatchX, iPatchZ);
		}
	}
}

void CTerrain::GenerateTerrainPatches(int32_t patchX, int32_t patchZ)
{
	const int32_t patchIndex = patchZ * PATCH_XCOUNT + patchX;
	auto& patch = m_vecTerrainPatches[patchIndex];

	if (!patch.IsUpdateNeeded())
	{
		return;
	}

	patch.Clear();
	patch.InitializeIndices();

	const int32_t startX = patchX * PATCH_XSIZE;
	const int32_t startZ = patchZ * PATCH_ZSIZE;

	float baseWorldX = static_cast<float>(m_iTerrCoordX * XSIZE * CELL_SCALE_METER) + static_cast<float>(startX * CELL_SCALE_METER);
	float baseWorldZ = static_cast<float>(m_iTerrCoordZ * ZSIZE * CELL_SCALE_METER) + static_cast<float>(startZ * CELL_SCALE_METER);

	float fPatchXSizeMeters = PATCH_XSIZE * CELL_SCALE_METER;
	float fPatchZSizeMeters = PATCH_ZSIZE * CELL_SCALE_METER;

	auto& vertices = patch.GetVertices();
	vertices.clear();
	vertices.reserve(PATCH_VERTEX_COUNT);

	Vector3D minBounds = { baseWorldX, FLT_MAX, baseWorldZ };
	Vector3D maxBounds = { baseWorldX + static_cast<float>(PATCH_XSIZE * CELL_SCALE_METER), FLT_MIN, baseWorldZ + static_cast<float>(PATCH_ZSIZE * CELL_SCALE_METER) };

	for (int32_t localZ = 0; localZ <= PATCH_ZSIZE; ++localZ)
	{
		for (int32_t localX = 0; localX <= PATCH_XSIZE; ++localX)
		{
			const int32_t mapX = startX + localX;
			const int32_t mapZ = startZ + localZ;

			const float h = 0.0f;

			STerrainVertex v{};
			v.m_v3Position = Vector3D{ baseWorldX + localX * CELL_SCALE_METER, h, baseWorldZ + localZ * CELL_SCALE_METER };

			v.m_v2TexCoords = Vector2D{ static_cast<float>(localX) / static_cast<float>(PATCH_XSIZE), static_cast<float>(localZ) / static_cast<float>(PATCH_ZSIZE) };

			const float tileX = static_cast<float>(mapX * HEIGHT_TILE_XRATIO);
			const float tileZ = static_cast<float>(mapZ * HEIGHT_TILE_ZRATIO);

			v.m_v2WeightUV = Vector2D{ tileX / static_cast<float>(TILEMAP_XSIZE), tileZ / static_cast<float>(TILEMAP_ZSIZE) };

			auto sampleHeight = [&](int32_t x, int32_t z) -> float
				{
					x = std::clamp(x, 0, HEIGHTMAP_RAW_XSIZE - 1);
					z = std::clamp(z, 0, HEIGHTMAP_RAW_ZSIZE - 1);
					return 0.0f;
				};
			float hL = sampleHeight(mapX - 1, mapZ);
			float hR = sampleHeight(mapX + 1, mapZ);
			float hD = sampleHeight(mapX, mapZ - 1);
			float hU = sampleHeight(mapX, mapZ + 1);

			Vector3D dx(2.0f * static_cast<float>(CELL_SCALE_METER), hR - hL, 0.0f);
			Vector3D dz(0.0f, hU - hD, 2.0f * static_cast<float>(CELL_SCALE_METER));
			Vector3D n = EngineMath::Normalize(EngineMath::Cross(dz, dx));
			v.m_v3Normals = n;

			v.m_v4Color = Vector4D(1.0f); // default white
			vertices.push_back(v);

		}
	}

	patch.SetIsWaterPatch(false);
	patch.SetPatchIndex(patchIndex);
	patch.SetUpdateNeed(false);
}

void CTerrain::AppendVisibleData(std::vector<SDrawElementsIndirectCommand>& globalCmds)
{
	for (auto& patch : m_vecTerrainPatches)
	{
		// Prepare Drawing command
		SDrawElementsIndirectCommand cmd{};

		// 1. How many indices to draw
		cmd.count = patch.GetIndexCount();
		cmd.instanceCount = 1;

		// Add the EBO bookmark
		cmd.firstIndex = patch.GetIndexOffset();

		// Tell the GPU where this terrain's vertices start in the VBO
		cmd.baseVertex = patch.GetVertexOffset();

		cmd.baseInstance = 0;

		globalCmds.push_back(cmd);
	}
}
