#pragma once

#include <cstdint>
#include <string>
#include <array>
#include <vector>
#include "TypeVector2.h"
#include "TypeVector3.h"
#include "TypeVector4.h"
#include "Grid.h"

#include <vulkan/vulkan.h>

class CVulkanTexture;

enum ETerrainChunkState : int32_t
{
	TERRAIN_CHUNK_STATE_UNLOADED = 0,		// Chunk is not loaded
	TERRAIN_CHUNK_STATE_LOADING = 1,		// Chunk is being loaded
	TERRAIN_CHUNK_STATE_LOADED = 2,			// Chunk is loaded and ready
	TERRAIN_CHUNK_STATE_ERROR = 3			// Error occurred during loading
};

enum ETerrainPatchData : int32_t
{
	PATCH_SIZE = 16,					// Number of quads in * direction per patch

	// Patch system (LOD/culling units)
	PATCH_XSIZE = PATCH_SIZE,										// Patch width in cells(e.g., 16 cells)
	PATCH_ZSIZE = PATCH_SIZE,										// Patch depth in cells (square patches)

	PATCH_VERTEX_COUNT = (PATCH_XSIZE + 1) * (PATCH_ZSIZE + 1), // Total vertices per patch
	PATCH_INDEX_COUNT = PATCH_XSIZE * PATCH_ZSIZE * 6 // Total indices per patch (2 triangles per quad)
};

enum ETerainData : int32_t
{
	TERRAIN_SIZE = 128,												// Number of quads in X and Z direction for the entire terrain

	TERRAIN_PATCHCOUNT = TERRAIN_SIZE / PATCH_XSIZE,				// Number of patches along each axis

	PATCH_XCOUNT = TERRAIN_PATCHCOUNT,								// Number of patches along X-axis
	PATCH_ZCOUNT = TERRAIN_PATCHCOUNT,								// Number of patches along Z-axis

	// Core terrain grid dimensions (in cells)
	XSIZE = TERRAIN_SIZE,											// Number of cells along X-axis (e.g., 128 cells)
	ZSIZE = TERRAIN_SIZE,											// Number of cells along Z-axis (matches X for square terrain)

	// Heightmap dimensions
	HEIGHTMAP_XSIZE = XSIZE + 1,									// Heightmap width: cells + 1 (vertices per row) and This ensures the heightmap covers all vertex points needed to define the terrain’s surface.
	HEIGHTMAP_ZSIZE = ZSIZE + 1,									// Heightmap depth: cells + 1 (vertices per row) and This ensures the heightmap covers all vertex points needed to define the terrain’s surface.
	HEIGHTMAP_SIZE = HEIGHTMAP_XSIZE * HEIGHTMAP_ZSIZE,				// Total heightmap samples

	// Heightmap raw dimensions (with padding for edge sampling)
	HEIGHTMAP_RAW_XSIZE = XSIZE + 3,								// Heightmap raw width: cells + 3 (vertices per row) (with padding for edge sampling)
	HEIGHTMAP_RAW_ZSIZE = ZSIZE + 3,								// Heightmap raw depth: cells + 3 (vertices per row) (prevents out-of-bounds access).
	HEIGHTMAP_RAW_SIZE = HEIGHTMAP_RAW_XSIZE * HEIGHTMAP_RAW_ZSIZE,	// Total heightmap samples

	// Tile maps (texture splatting)
	TILEMAP_XSIZE = XSIZE * 2,										// Tile map width (2 tiles per cell for texture variation)
	TILEMAP_ZSIZE = ZSIZE * 2,										// Tile map depth (e.g., 256x256 for 128-cell terrain)
	TILEMAP_RAW_XSIZE = (XSIZE) * 2,								// Raw tile map width (with padding for filtering)
	TILEMAP_RAW_ZSIZE = (ZSIZE) * 2,								// Raw tile map depth (avoids edge artifacts)

	// Tile-to-heightmap ratios
	HEIGHT_TILE_XRATIO = TILEMAP_XSIZE / XSIZE,						// Tiles per cell width (2:1)
	HEIGHT_TILE_ZRATIO = TILEMAP_ZSIZE / ZSIZE,						// Tiles per cell depth (2:1)

	// Spatial scaling
	CELL_SCALE = 200,												// Cell size in centimeters (e.g., 200cm = 2m)
	CELL_SCALE_METER = CELL_SCALE / 100,							// Cell size in meters (e.g 200cm = 2m)
	HALF_CELL_SCALE = CELL_SCALE / 2,								// Half-cell offset (for centering objects)
	HALF_CELL_SCALE_METER = HALF_CELL_SCALE / 100,					// Half-cell offset in meters (1m)

	// Water Mapping
	MAX_WATER_NUM = 255,											// Maximum number of water bodies per terrain patch
	WATERMAP_XSIZE = XSIZE,											// Water patch width in cells (e.g., 256 cells)
	WATERMAP_ZSIZE = ZSIZE,											// Water patch depth in cells (e.g., 256 cells)

	HEIGHT_WATER_XRATIO = WATERMAP_XSIZE / XSIZE,					// Tiles per cell width (2:1)
	HEIGHT_WATER_ZRATIO = WATERMAP_ZSIZE / ZSIZE,					// Tiles per cell depth (2:1)

	// Attribute maps (e.g., collision, material types)
	ATTRMAP_XSIZE = TILEMAP_XSIZE,									// Attribute map width (2x resolution for per-corner data)
	ATTRMAP_ZSIZE = TILEMAP_ZSIZE,									// Attribute map depth (stores data at cell corners)
	ATTRMAP_COUNT = 8,												// Number of attribute layers (e.g., 8 block, no PVP, water, etc)

	// Terrain Actual size
	TERRAIN_XSIZE = XSIZE * CELL_SCALE_METER,						// Total terrain width in meters
	TERRAIN_ZSIZE = ZSIZE * CELL_SCALE_METER,						// Total terrain depth in meters
};

enum EMapOutdoorData
{
	TERRAIN_LOAD_SIZE = 1,
	MAX_RENDER_TERRAINS_NUM = 1 + (TERRAIN_LOAD_SIZE * 2) * (TERRAIN_LOAD_SIZE * 2) * 2,
	MAX_TERRAIN_DATA_LENGTH = 256,
};

using TerrainMaterialID = uint32_t;
static constexpr TerrainMaterialID INVALID_TERRAIN_MATERIAL = 0xFFFFFFFF;
static constexpr TerrainMaterialID TERRAIN_CHUNK_MAX_LAYERS = 4;

struct STerrainMaterial
{
	TerrainMaterialID id = INVALID_TERRAIN_MATERIAL;
	std::string name = "";

	std::string albedoPath = "";
	std::string normalPath = "";
	std::string ormPath = "";      // occlusion/roughness/metallic, or roughness-only if simpler
	std::string heightPath = "";   // optional, for later height blending

	CVulkanTexture* pAlbedo = nullptr;
	CVulkanTexture* pNormal = nullptr;
	CVulkanTexture* pORM = nullptr;
	CVulkanTexture* pHeight = nullptr;

	float uvScale = 8.0f;
	float normalStrength = 1.0f;
	float roughnessMul = 1.0f;
	Vector3D tint = Vector3D(1.0f, 1.0f, 1.0f);
	bool triplanar = false;
};

struct SWeightTexel
{
	uint8_t r, g, b, a;
	uint8_t& operator[](size_t index)
	{
		switch (index)
		{
		case 0:
			return r;
		case 1:
			return g;
		case 2:
			return b;
		case 3:
		default:
			return r; // Fallback for invalid index
		}
	}
};

struct STerrainChunkLayers
{
	std::array<TerrainMaterialID, TERRAIN_CHUNK_MAX_LAYERS> materialIDs =
	{
		INVALID_TERRAIN_MATERIAL,
		INVALID_TERRAIN_MATERIAL,
		INVALID_TERRAIN_MATERIAL,
		INVALID_TERRAIN_MATERIAL
	};

	CVulkanTexture* pWeightMap = nullptr;     // RGBA8
	uint32_t weightMapResolution = TILEMAP_XSIZE;

	// std::vector<uint8_t> weightMapPixels; // RGBA8 CPU copy
	// std::vector<SWeightTexel> weightMapPixels; // RGBA8 CPU copy
	CGrid<SWeightTexel> weightMapPixels;

	bool dirty = false;

	uint32_t dirtyMinX = 0;
	uint32_t dirtyMinY = 0;
	uint32_t dirtyMaxX = 0;
	uint32_t dirtyMaxY = 0;
};

struct STerrainVertex
{
	Vector3D m_v3Position;		// World position
	Vector3D m_v3Normals;		// Normal
	Vector2D m_v2TexCoords;		// UVs per Patch (For Texturing)
	Vector2D m_v2WeightUV;		// UVs Per Terrain whole (For Texturing)
	Vector4D m_v4Color;			// Color

	STerrainVertex() = default;

	STerrainVertex(
		const SVector3Df& inPosition,
		const SVector3Df& inNormal,
		const SVector2Df& inTexCoord,
		const SVector2Df& inWeightUV,
		const SVector4Df& inColor = SVector4Df(1.0f))
		: m_v3Position(inPosition)
		, m_v3Normals(inNormal)
		, m_v2TexCoords(inTexCoord)
		, m_v2WeightUV(inWeightUV)
		, m_v4Color(inColor)
	{
	}

	static VkVertexInputBindingDescription GetBindingDescription()
	{
		VkVertexInputBindingDescription bindingDescription{};
		bindingDescription.binding = 0;
		bindingDescription.stride = sizeof(STerrainVertex);
		bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
		return bindingDescription;
	}

	static std::vector<VkVertexInputAttributeDescription> GetAttributeDescriptions()
	{
		static std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};
		attributeDescriptions.resize(5); // 5 attributes

		attributeDescriptions[0].binding = 0;
		attributeDescriptions[0].location = 0;
		attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[0].offset = offsetof(STerrainVertex, STerrainVertex::m_v3Position);

		attributeDescriptions[1].binding = 0;
		attributeDescriptions[1].location = 1;
		attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
		attributeDescriptions[1].offset = offsetof(STerrainVertex, STerrainVertex::m_v3Normals);

		attributeDescriptions[2].binding = 0;
		attributeDescriptions[2].location = 2;
		attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
		attributeDescriptions[2].offset = offsetof(STerrainVertex, STerrainVertex::m_v2TexCoords);

		attributeDescriptions[3].binding = 0;
		attributeDescriptions[3].location = 3;
		attributeDescriptions[3].format = VK_FORMAT_R32G32_SFLOAT;
		attributeDescriptions[3].offset = offsetof(STerrainVertex, STerrainVertex::m_v2WeightUV);

		attributeDescriptions[4].binding = 0;
		attributeDescriptions[4].location = 4;
		attributeDescriptions[4].format = VK_FORMAT_R32G32B32A32_SFLOAT;
		attributeDescriptions[4].offset = offsetof(STerrainVertex, STerrainVertex::m_v4Color);

		return attributeDescriptions;
	}
};

struct SDrawElementsIndirectCommand
{
	uint32_t  count;         // Indices in patch
	uint32_t  instanceCount; // 1
	uint32_t  firstIndex;    // Offset in EBO
	int32_t   baseVertex;    // Offset in VBO
	uint32_t  baseInstance;  // 0
};

using DrawElementsIndirectCommand = SDrawElementsIndirectCommand;

// Terrain Data Script Types
#define TERRAIN_PROPERTIES_SCRIPT "AnubisTerrainProperties"
#define MAP_SETTINGS_SCRIPT "AnubisMapSettings"
#if defined(_WIN32) || defined(_WIN64)
#define MAP_ASSET_PATH "Assets\\Maps\\"
#else
#define MAP_ASSET_PATH "Assets/Maps/"
#endif