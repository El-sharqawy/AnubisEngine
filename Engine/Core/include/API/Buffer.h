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

enum class EBindingLayoutSetsPoints : uint32_t
{
	BINDING_POINT_SET_MATERIAL,         // SET 0
	BINDING_POINT_SET_FRAME_RESOURCES,  // SET 1
	BINDING_POINT_SET_BONES_RESOURCES,  // SET 2
	BINDING_POINT_SET_MAX_NUM,
};

// Textures Binding Points
enum class EMaterialBindingSets : uint32_t
{
	BINDING_POINT_MATERIAL_SET_SAMPLER_0,
	BINDING_POINT_MATERIAL_SET_SAMPLER_1,
	BINDING_POINT_MATERIAL_SET_SAMPLER_2,
	BINDING_POINT_MATERIAL_SET_SAMPLER_3,
	BINDING_POINT_MATERIAL_SET_SAMPLER_4,
	BINDING_POINT_MATERIAL_SET_SAMPLER_5,
	BINDING_POINT_MATERIAL_SET_MAX,
};

// Buffers Binding Points
enum class EUniformBuffersBindingSets : uint32_t
{
	BINDING_POINT_UBO_CAMERA,
	BINDING_POINT_UBO_MODEL, // OpenGL
	BINDING_POINT_UBO_MAX,
};

enum class EStorageBufferBindingSets : uint32_t
{
	BINDING_POINT_BONES_SSBO,
};

struct SBindingSets
{
	EBindingLayoutSetsPoints bindingSet = EBindingLayoutSetsPoints::BINDING_POINT_SET_MAX_NUM;
	uint32_t bindingPoint = UINT32_MAX; // only meaningful for UBO/SSBO, UINT32_MAX = not bound to any base

	bool IsValid() const
	{
		return bindingSet != EBindingLayoutSetsPoints::BINDING_POINT_SET_MAX_NUM && bindingPoint != UINT32_MAX;
	}

	uint32_t GetPackedKey() const
	{
		return (static_cast<uint32_t>(bindingSet) << 16) | bindingPoint;
	}
};

struct SBufferDesc
{
	std::string m_stName = "Buffer";
	EBufferType m_eType = EBufferType::BUFFER_TYPE_VERTEX;
	EBufferMemoryType m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY;
	SBindingSets m_sBindingSets{};
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
	virtual EBindingLayoutSetsPoints GetBindingLayoutSetsPoint() const = 0;
	virtual uint32_t GetBindingPoint() const = 0;
	virtual uint64_t GetSize() const = 0;
	virtual bool IsValid() const = 0;

protected:
	// Buffer Properties
	std::string m_stName = "Buffer";
	EBufferType m_eType = EBufferType::BUFFER_TYPE_VERTEX;
	EBufferMemoryType m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY;
	SBindingSets m_sBindingSets{};
	uint64_t m_uiSize = 0;
	bool m_bIsValid = false;
};

