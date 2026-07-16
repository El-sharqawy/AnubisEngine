#pragma once

#include <vulkan/vulkan.h>
#include <cstdint>

constexpr size_t MAX_MAIN_VRAM_BYTES = 1024 * 1024 * 1024;	// 1GB for main buffers
constexpr size_t MAX_DEBUG_VRAM_BYTES = 256 * 1024 * 1024;	// 256MB for debug buffers

constexpr uint32_t INITIAL_BONE_CAPACITY = 512;

enum class EBufferResizeResult
{
	BUFFER_RESIZE_UNCHANGED,
	BUFFER_RESIZE_RECREATED,
	BUFFER_RESIZE_FAILED,
};

enum class EBufferType
{
	BUFFER_TYPE_UNIFORM,		// UBO
	BUFFER_TYPE_STORAGE,		// SSBO
	BUFFER_TYPE_INDEX,			// Index
	BUFFER_TYPE_VERTEX,			// Vertex
	BUFFER_TYPE_INDIRECT,		// Indirect Buffer
	BUFFER_TYPE_TRANSFER_SRC,
	BUFFER_TYPE_TRANSFER_DST,
	BUFFER_TYPE_STAGING,
	BUFFER_MAX_TYPE,
};

enum class EBufferMemoryType
{
	BUFFER_MEMORY_GPU_ONLY,
	BUFFER_MEMORY_CPU_WRITE,
	BUFFER_MEMORY_CPU_READ,
	BUFFER_MEMORY_CPU_READ_WRITE,
};

// Textures Binding Points
enum class EBufferBindingPointsSetZero
{
	BINDING_POINT_SET_ZERO_SAMPLER_0, // Not bound to any point
	BINDING_POINT_SET_ZERO_SAMPLER_1,
	BINDING_POINT_SET_ZERO_SAMPLER_2,
	BINDING_POINT_SET_ZERO_SAMPLER_3,
	BINDING_POINT_SET_ZERO_SAMPLER_4,
	BINDING_POINT_SET_ZERO_SAMPLER_5,
	BINDING_POINT_SET_ZERO_MAX,
};

// Buffers Binding Points
enum class EBufferBindingPointsSetOne
{
	BINDING_POINT_SET_ONE_CAMERA_UBO,
	BINDING_POINT_SET_ONE_BONES_SSBO,
	BINDING_POINT_SET_ONE_MODEL_UBO,
	BINDING_POINT_SET_ONE_MAX,
};

struct SBufferDesc
{
	std::string m_stName = "Buffer";
	EBufferType m_eType = EBufferType::BUFFER_TYPE_VERTEX;
	EBufferMemoryType m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY;
	EBufferBindingPointsSetZero m_eBindingPointZero = EBufferBindingPointsSetZero::BINDING_POINT_SET_ZERO_MAX;
	EBufferBindingPointsSetOne m_eBindingPointOne = EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_MAX;

	uint64_t m_uiSize = 0;
	bool cpuWrite = false;
};

class IBuffer
{
public:
	virtual ~IBuffer() = default;
	virtual const std::string& GetName() const = 0;
	virtual EBufferType GetType() const = 0;
	virtual EBufferMemoryType GetMemoryType() const = 0;
	virtual uint64_t GetSize() const = 0;
	virtual bool IsValid() const = 0;

protected:
	// Buffer Properties
	std::string m_stName = "Buffer";
	EBufferType m_eType = EBufferType::BUFFER_TYPE_VERTEX;
	EBufferMemoryType m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY;
	uint64_t m_uiSize = 0;
	bool m_bIsValid = false;
};

