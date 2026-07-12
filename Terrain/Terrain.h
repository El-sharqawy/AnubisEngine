#pragma once

#include "TerrainPatch.h"

class CTerrainMap;

class CTerrain
{
public:
	CTerrain() = default;
	~CTerrain() = default;

	void Initialize();
	void Clear();

	void GenerateTerrainPatches();
	void GenerateTerrainPatches(int32_t iPatchNumX, int32_t iPatchNumZ);
	void AppendVisibleData(std::vector<SDrawElementsIndirectCommand>& globalCmds);

	// Terrain Accessors
public:
	const std::string& GetName() const;
	void SetName(const std::string& stName);

	bool IsReady() const;
	void SetReady(bool bReady);

	int32_t GetTerrainNumber() const;
	void SetTerrainNumber(int32_t iTerrainNum);

	void GetTerrainCoords(int32_t* ipX, int32_t* ipZ) const;
	void SetTerrainCoords(int32_t iX, int32_t iZ);

	int32_t GetTerrainXCoord() const;
	int32_t GetTerrainZCoord() const;
	void SetTerrainXCoord(int32_t iX);
	void SetTerrainZCoord(int32_t iZ);

	CTerrainMap* GetParentMap() const;
	void SetParentMap(CTerrainMap* pMap);

private:
	// Terrain Patches
	std::vector<CTerrainPatch> m_vecTerrainPatches;

	// Terrain Name
	std::string m_stTerrainName = "AreaTerrain";

	// Is Terrain Ready
	bool m_bReady = false;

	// Terrain Number
	int32_t m_iTerrainNum = 0;

	// Terrain Coordinates
	int32_t m_iTerrCoordX = 0;
	int32_t m_iTerrCoordZ = 0;

	// Parent Terrain Map
	CTerrainMap* m_pParentMap = nullptr;
};