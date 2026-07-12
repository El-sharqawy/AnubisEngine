#pragma once

#include "API/Buffer.h"
#include <string>

// Matches our GLSL layout(location = N)
enum class EVertexAttribute : uint32_t
{
	Position = 0,
	Normal = 1,
	TexCoord0 = 2,
	TexCoord1 = 3,
	Tangent = 4,
	Bitangent = 5,
	Color = 6,
	BoneIDs = 7,
	Weights = 8,
};

enum class EVertexAttribClass : uint8_t
{
	Float,      // glVertexAttribPointer / glVertexArrayAttribFormat
	Integer,    // glVertexAttribIPointer / glVertexArrayAttribIFormat
	Double      // glVertexAttribLPointer / glVertexArrayAttribLFormat
};

struct SVertexAttribDesc
{
	uint32_t				m_uiLocation = 0;          // shader location
	uint32_t				m_uiBinding = 0;          // vertex buffer binding slot
	int32_t					m_iCount = 0;          // 1..4
	uint32_t				m_eDataType = GL_FLOAT;   // GL_FLOAT, GL_UNSIGNED_INT, etc.
	EVertexAttribClass		m_eClass = EVertexAttribClass::Float;
	bool					m_bNormalize = false;   // only meaningful for Float path
	uint32_t				m_uiOffset = 0;          // byte offset inside bound stream
	uint32_t				m_uiDivisor = 0;          // 0 = per-vertex, >0 = instanced
};

struct SVertexBufferBindingDesc
{
	IBuffer* m_pBuffer = nullptr;
	uint32_t			m_uiBinding = 0;
	uint32_t			m_uiOffset = 0;
	int32_t				m_iStride = 0;
	uint32_t			m_uiDivisor = 0;               // per-binding divisor for DSA path
};

struct SVertexArrayDesc
{
	std::string                             m_stName;
	std::vector<SVertexBufferBindingDesc>   m_vBindings;
	std::vector<SVertexAttribDesc>          m_vAttribs;
	IBuffer* m_pIndexBuffer = nullptr;
	uint32_t                                  m_eIndexType = GL_UNSIGNED_INT;
};

struct SVertexArrayData
{
	std::string m_stName;
	uint32_t m_uiVaoID = 0;
	uint32_t m_uiVertexCount = 0;
	uint32_t m_uiIndexCount = 0;
	uint32_t m_eIndexType = GL_UNSIGNED_INT;
	bool m_bHasIndexBuffer = false;
};

class CVertexArray
{
public:
    CVertexArray() = default;
    virtual ~CVertexArray() = default;

    // Non-copyable, movable ?
    CVertexArray(const CVertexArray&) = delete;
    CVertexArray& operator=(const CVertexArray&) = delete;

    bool IsValid() const;
    uint32_t GetID() const;
    bool HasIndexBuffer() const;
    uint32_t GetIndexType() const;
    uint32_t GetIndexCount() const;
    uint32_t GetVertexCount() const;
    const std::string& GetName() const;

	void UpdateVertexArrayData(const SVertexArrayData& vertexArrayData);

private:
    std::string m_stName;
    uint32_t m_uiVaoID = 0;
    uint32_t m_uiVertexCount = 0;
    uint32_t m_uiIndexCount = 0;
    uint32_t m_eIndexType = GL_UNSIGNED_INT;
    bool m_bHasIndexBuffer = false;
};