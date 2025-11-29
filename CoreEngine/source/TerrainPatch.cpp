#include "TerrainPatch.h"

CTerrainPatch::CTerrainPatch()
{
	Initialize();
}

CTerrainPatch::~CTerrainPatch()
{
	Clear();
}

void CTerrainPatch::Initialize()
{
	// patch properties
	m_iPatchWidth = 0;
	m_iPatchDepth = 0;

	// OpenGL properties
	m_uiVAO = 0;
	m_uiVBO = 0;
	m_uiEBO = 0;
}

void CTerrainPatch::Clear()
{
	// patch properties
	m_iPatchWidth = 0;
	m_iPatchDepth = 0;

	// OpenGL properties
	if (m_uiVAO)
	{
		glDeleteVertexArrays(1, &m_uiVAO);
		m_uiVAO = 0;
	}
	if (m_uiVBO)
	{
		glDeleteBuffers(1, &m_uiVBO);
		m_uiVBO = 0;
	}
	if (m_uiEBO)
	{
		glDeleteBuffers(1, &m_uiEBO);
		m_uiEBO = 0;
	}

	m_vecVertices.clear();
	m_vecIndices.clear();
}

void CTerrainPatch::InitializePatch()
{
	m_iPatchWidth = ETerrainData::PATCH_XSIZE + 1;
	m_iPatchDepth = ETerrainData::PATCH_ZSIZE + 1;

	InitializeVertices();
	InitializeIndices();

	InitializeOpenGLData();
}

void CTerrainPatch::InitializeVertices()
{
	m_vecVertices.reserve(PATCH_VERTEX_COUNT);

	for (GLint iZ = 0; iZ < m_iPatchDepth; iZ++)
	{
		for (GLint iX = 0; iX < m_iPatchWidth; iX++)
		{
			GLfloat fX = static_cast<GLfloat>(iX * CELL_SCALE);
			GLfloat fY = 0.0f;
			GLfloat fZ = static_cast<GLfloat>(iZ * CELL_SCALE);


			TerrainVertex vertex{};
			vertex.m_v3Position = Vector3D(fX, fY, fZ);
			vertex.m_v2TexCoords = Vector2D(iX / PATCH_XSIZE, iZ / PATCH_ZSIZE);
			vertex.m_v3Normals = Vector3D(0.0f);
				
			m_vecVertices.push_back(vertex);
		}
	}

	assert(m_vecVertices.size() == PATCH_VERTEX_COUNT);
}

void CTerrainPatch::InitializeIndices()
{
	m_vecIndices.reserve(PATCH_XSIZE * PATCH_ZSIZE * 6);

	for (GLint iZ = 0; iZ < m_iPatchDepth - 1; iZ++)
	{
		for (GLint iX = 0; iX < m_iPatchWidth - 1; iX++)
		{
			GLint iTopLeft = iZ * m_iPatchWidth + iX;
			GLint iTopRight = iZ * m_iPatchWidth + (iX + 1);
			GLint iBottomLeft = (iZ + 1) * m_iPatchWidth + iX;
			GLint iBottomRight = (iZ + 1) * m_iPatchWidth + (iX + 1);

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

void CTerrainPatch::InitializeOpenGLData()
{
	// create Vertex Array
	glCreateVertexArrays(1, &m_uiVAO);

	// create vertex buffer object
	glCreateBuffers(1, &m_uiVBO);

	const GLsizeiptr vertexBufferSize = m_vecVertices.size() * sizeof(TerrainVertex);
	glNamedBufferStorage(m_uiVBO, vertexBufferSize, m_vecVertices.data(), GL_MAP_WRITE_BIT | GL_DYNAMIC_STORAGE_BIT);

	// create element buffer object
	glCreateBuffers(1, &m_uiEBO);

	const GLsizeiptr indexBufferSize = m_vecIndices.size() * sizeof(GLuint);
	glNamedBufferStorage(m_uiEBO, indexBufferSize, m_vecIndices.data(), GL_MAP_WRITE_BIT | GL_DYNAMIC_STORAGE_BIT);

	// Attach Buffers to our Vertex Arrays
	glVertexArrayVertexBuffer(m_uiVAO, 0, m_uiVBO, 0, sizeof(TerrainVertex)); // attach vertex buffer
	glVertexArrayElementBuffer(m_uiVAO, m_uiEBO); // element buffer

	// vertex array attributes
	glEnableVertexArrayAttrib(m_uiVAO, 0);
	glVertexArrayAttribFormat(m_uiVAO, 0, 3, GL_FLOAT, GL_FALSE, offsetof(TerrainVertex, m_v3Position)); // Position Attribute
	glVertexArrayAttribBinding(m_uiVAO, 0, 0);

	glEnableVertexArrayAttrib(m_uiVAO, 1);
	glVertexArrayAttribFormat(m_uiVAO, 1, 2, GL_FLOAT, GL_FALSE, offsetof(TerrainVertex, m_v2TexCoords)); // textures coords Attribute
	glVertexArrayAttribBinding(m_uiVAO, 1, 0);

	glEnableVertexArrayAttrib(m_uiVAO, 2);
	glVertexArrayAttribFormat(m_uiVAO, 2, 3, GL_FLOAT, GL_FALSE, offsetof(TerrainVertex, m_v3Normals)); // Normals Attribute
	glVertexArrayAttribBinding(m_uiVAO, 2, 0);
}
