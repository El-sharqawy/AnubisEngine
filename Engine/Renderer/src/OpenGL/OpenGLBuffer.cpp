#include "OpenGL/OpenGLBuffer.h"
#include "Logging/LogManager.h"

static constexpr size_t SLICE_ALIGNMENT = 256;
static constexpr size_t SUB_ALIGNMENT = 16;

COpenGLBuffer::COpenGLBuffer(COpenGLBuffer&& other) noexcept
{
	m_uiBufferID = std::exchange(other.m_uiBufferID, 0);

	m_stName = std::move(other.m_stName);
	m_eType = std::exchange(other.m_eType, EBufferType::BUFFER_TYPE_VERTEX);
	m_eMemoryType = std::exchange(other.m_eMemoryType, EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY);
	m_sBindingSets.bindingSet = std::exchange(other.m_sBindingSets.bindingSet, EBindingLayoutSetsPoints::BINDING_POINT_SET_MAX_NUM);
	m_sBindingSets.bindingPoint = std::exchange(other.m_sBindingSets.bindingPoint, UINT32_MAX);
	m_uiSize = std::exchange(other.m_uiSize, 0);
	m_bIsValid = std::exchange(other.m_bIsValid, false);
}

COpenGLBuffer& COpenGLBuffer::operator=(COpenGLBuffer&& other) noexcept
{
	if (this == &other)
	{
		return *this;
	}

	// Destroy our existing GL resource first
	if (m_uiBufferID != 0)
	{
		glDeleteBuffers(1, &m_uiBufferID);
	}

	m_uiBufferID = std::exchange(other.m_uiBufferID, 0);

	m_stName = std::move(other.m_stName);
	m_eType = std::exchange(other.m_eType, EBufferType::BUFFER_TYPE_VERTEX);
	m_eMemoryType = std::exchange(other.m_eMemoryType, EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY);
	m_sBindingSets.bindingSet = std::exchange(other.m_sBindingSets.bindingSet, EBindingLayoutSetsPoints::BINDING_POINT_SET_MAX_NUM);
	m_sBindingSets.bindingPoint = std::exchange(other.m_sBindingSets.bindingPoint, UINT32_MAX);
	m_uiSize = std::exchange(other.m_uiSize, 0);
	m_bIsValid = std::exchange(other.m_bIsValid, false);

	return (*this);
}

void COpenGLBuffer::Clear()
{
	m_uiBufferID = 0;

    m_stName = "Buffer";
    m_eType = EBufferType::BUFFER_TYPE_VERTEX;
    m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY;
	m_sBindingSets.bindingSet = EBindingLayoutSetsPoints::BINDING_POINT_SET_MAX_NUM;
	m_sBindingSets.bindingPoint = UINT32_MAX;
	m_uiSize = 0;
    m_bIsValid = false;
}

void COpenGLBuffer::UpdateBufferData(const SBufferDesc& bufferDesc, uint32_t uiBufferID)
{
	m_uiBufferID = uiBufferID;

	m_stName = bufferDesc.m_stName;
	m_eType = bufferDesc.m_eType;
	m_eMemoryType = bufferDesc.m_eMemoryType;
	m_sBindingSets.bindingSet = bufferDesc.m_sBindingSets.bindingSet;
	m_sBindingSets.bindingPoint = bufferDesc.m_sBindingSets.bindingPoint;

	m_uiSize = bufferDesc.m_uiSize;
	m_bIsValid = true;
}

bool COpenGLBuffer::IsBoundToBase() const
{
	return GetBindingPoint() != UINT32_MAX && GetBindingLayoutSetsPoint() != EBindingLayoutSetsPoints::BINDING_POINT_SET_MAX_NUM;
}

void COpenGLBuffer::UpdateBufferID(uint32_t uiNewBufferID)
{
	m_uiBufferID = uiNewBufferID;
}

void COpenGLBuffer::UpdateBufferSize(uint64_t uiNewBufferSize)
{
	m_uiSize = uiNewBufferSize;
}
