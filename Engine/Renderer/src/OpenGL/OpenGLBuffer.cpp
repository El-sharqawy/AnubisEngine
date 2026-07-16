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
	m_eBindingPoint = std::exchange(other.m_eBindingPoint, EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_MAX);
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
	m_eBindingPoint = std::exchange(other.m_eBindingPoint, EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_MAX);
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
	m_eBindingPoint = EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_MAX;
    m_uiSize = 0;
    m_bIsValid = false;
}

void COpenGLBuffer::UpdateBufferData(const SBufferDesc& bufferDesc, uint32_t uiBufferID)
{
	m_uiBufferID = uiBufferID;

	m_stName = bufferDesc.m_stName;
	m_eType = bufferDesc.m_eType;
	m_eMemoryType = bufferDesc.m_eMemoryType;
	m_eBindingPoint = bufferDesc.m_eBindingPointOne;

	m_uiSize = bufferDesc.m_uiSize;
	m_bIsValid = true;
}

void COpenGLBuffer::SetBindingPoint(EBufferBindingPointsSetOne eBindingPt)
{
	m_eBindingPoint = eBindingPt;
}

bool COpenGLBuffer::IsBoundToBase() const
{
	return m_eBindingPoint != EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_MAX;
}

void COpenGLBuffer::UpdateBufferID(uint32_t uiNewBufferID)
{
	m_uiBufferID = uiNewBufferID;
}

void COpenGLBuffer::UpdateBufferSize(uint64_t uiNewBufferSize)
{
	m_uiSize = uiNewBufferSize;
}
