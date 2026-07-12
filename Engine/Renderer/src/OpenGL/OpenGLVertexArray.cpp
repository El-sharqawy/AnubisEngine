#include "OpenGL/OpenGLVertexArray.h"

bool CVertexArray::IsValid() const
{
    return (m_uiVaoID != 0);
}

uint32_t CVertexArray::GetID() const
{
    return (m_uiVaoID);
}

bool CVertexArray::HasIndexBuffer() const
{
    return (m_bHasIndexBuffer);
}

uint32_t CVertexArray::GetIndexType() const
{
    return (m_eIndexType);
}

uint32_t CVertexArray::GetIndexCount() const
{
    return (m_uiIndexCount);
}

uint32_t CVertexArray::GetVertexCount() const
{
    return (m_uiVertexCount);
}

const std::string& CVertexArray::GetName() const
{
    return (m_stName);
}

void CVertexArray::UpdateVertexArrayData(const SVertexArrayData& vertexArrayData)
{
    m_stName = vertexArrayData.m_stName;
    m_uiVaoID = vertexArrayData.m_uiVaoID;
    m_uiVertexCount = vertexArrayData.m_uiVertexCount;
    m_uiIndexCount = vertexArrayData.m_uiIndexCount;
    m_eIndexType = vertexArrayData.m_eIndexType;
    m_bHasIndexBuffer = vertexArrayData.m_bHasIndexBuffer;
}
