#pragma once

#include <maths.h>
#include <vector>

enum ETerrainData
{
	PATCH_XSIZE = 16,
	PATCH_ZSIZE = 16,

	PATCH_VERTEX_COUNT = (PATCH_XSIZE + 1) * (PATCH_ZSIZE + 1),

	CELL_SCALE = 2,
};

class CTerrainPatch
{
public:
	CTerrainPatch();
	~CTerrainPatch();

	void Initialize();
	void Clear();

	void InitializePatch();

protected:
	void InitializeVertices();
	void InitializeIndices();
	void InitializeOpenGLData();

private:
	// patch properties
	GLint m_iPatchWidth;
	GLint m_iPatchDepth;

	// OpenGL properties
	GLuint m_uiVAO;
	GLuint m_uiVBO;
	GLuint m_uiEBO;

	// patch vertex data
	std::vector<TerrainVertex> m_vecVertices;
	std::vector<GLuint> m_vecIndices;
};