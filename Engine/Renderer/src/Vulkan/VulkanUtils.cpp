#include "Vulkan/VulkanUtils.h"

bool VulkanUtils::VulkanFailed(VkResult result, const char* expr, const char* file, int line)
{
	if (result != VK_SUCCESS)
	{
		syserr("VkResult Error: {} from {} in {} at line {}", string_VkResult(result), expr, file, line);
		return false;
	}
	return true;
}

std::vector<uint32_t> VulkanUtils::ReadShaderData(const std::string& filename)
{
	// Open at the end (ios::ate) and in binary mode
	std::ifstream file(filename, std::ios::ate | std::ios::binary);

	if (!file.is_open())
	{
		syserr("Failed to Open the file {}", filename.c_str());
		return {};
	}

	size_t fileSize = static_cast<size_t>(file.tellg());

	if (fileSize % sizeof(uint32_t) != 0)
	{
		printf("SPIR-V file size is not a multiple of 4: {}", filename.c_str());
		file.close();
		return {};
	}

	// Crucial for Vulkan SPIR-V: Ensure the vector data size accounts for 4-byte chunks
	// and is naturally aligned to uint32_t elements
	size_t reserveSize = fileSize / sizeof(uint32_t);
	std::vector<uint32_t> buffer(reserveSize);

	file.seekg(0);
	// Read directly into the uint32_t buffer data pointer safely casted
	if (!file.read(reinterpret_cast<char*>(buffer.data()), fileSize))
	{
		printf("Failed to read shader file: %s\n", filename.c_str());
		file.close();
		return {};
	}

	file.close();

	return buffer;
}

VkPrimitiveTopology VulkanUtils::ToVkTopology(EPrimitiveTopology topology)
{
	switch (topology)
	{
	case EPrimitiveTopology::TOPOLOGY_POINTS:
		return (VK_PRIMITIVE_TOPOLOGY_POINT_LIST);
	case EPrimitiveTopology::TOPOLOGY_LINES_LIST:
		return (VK_PRIMITIVE_TOPOLOGY_LINE_LIST);
	case EPrimitiveTopology::TOPOLOGY_LINES_STRIP:
		return (VK_PRIMITIVE_TOPOLOGY_LINE_STRIP);
	case EPrimitiveTopology::TOPOLOGY_TRIANGLES_LIST:
		return (VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
	case EPrimitiveTopology::TOPOLOGY_TRIANGLES_STRIP:
		return (VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP);
	case EPrimitiveTopology::TOPOLOGY_TRIANGLES_FAN:
		return (VK_PRIMITIVE_TOPOLOGY_TRIANGLE_FAN);
	case EPrimitiveTopology::TOPOLOGY_QUADS:
		return (VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
	default:
		return (VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST);
	}
}

VkCullModeFlags VulkanUtils::ToVkCullMode(ECullMode mode)
{
	switch (mode)
	{
	case ECullMode::CULL_MODE_NONE:
		return (VK_CULL_MODE_NONE);
	case ECullMode::CULL_MODE_BACK:
		return (VK_CULL_MODE_BACK_BIT);
	case ECullMode::CULL_MODE_FRONT:
		return (VK_CULL_MODE_FRONT_BIT);
	default:
		return (VK_CULL_MODE_BACK_BIT);
	}
}

VkFrontFace VulkanUtils::ToVkFrontFace(EFrontFace face)
{
	switch (face)
	{
	case EFrontFace::FRONT_FACE_CLOCKWISE:
		return (VK_FRONT_FACE_CLOCKWISE);
	case EFrontFace::FRONT_FACE_COUNTER_CLOCKWISE:
		return (VK_FRONT_FACE_COUNTER_CLOCKWISE);
	default:
		return (VK_FRONT_FACE_COUNTER_CLOCKWISE);
	}
}

VkPolygonMode VulkanUtils::ToVkPolygonMode(EPolygonMode polygonMode)
{
	switch (polygonMode)
	{
	case EPolygonMode::POLYGON_MODE_FILL:
		return (VK_POLYGON_MODE_FILL);
	case EPolygonMode::POLYGON_MODE_LINE:
		return (VK_POLYGON_MODE_LINE);
	case EPolygonMode::POLYGON_MODE_POINT:
		return (VK_POLYGON_MODE_LINE);
	default:
		return (VK_POLYGON_MODE_FILL);
	}
}

VkCompareOp VulkanUtils::ToVkCompareOp(EDepthCompareOp op)
{
	switch (op)
	{
	case EDepthCompareOp::DEPTH_NEVER:
		return VK_COMPARE_OP_NEVER;
	case EDepthCompareOp::DEPTH_LESS:
		return VK_COMPARE_OP_LESS;
	case EDepthCompareOp::DEPTH_EQUAL:
		return VK_COMPARE_OP_EQUAL;
	case EDepthCompareOp::DEPTH_LESS_OR_EQUAL:
		return VK_COMPARE_OP_LESS_OR_EQUAL;
	case EDepthCompareOp::DEPTH_GREATER:
		return VK_COMPARE_OP_GREATER;
	case EDepthCompareOp::DEPTH_NOT_EQUAL:
		return VK_COMPARE_OP_NOT_EQUAL;
	case EDepthCompareOp::DEPTH_GREATER_OR_EQUAL:
		return VK_COMPARE_OP_GREATER_OR_EQUAL;
	case EDepthCompareOp::DEPTH_ALWAYS:
		return VK_COMPARE_OP_ALWAYS;
	default:
		return VK_COMPARE_OP_LESS;
	}
}

VkIndexType VulkanUtils::ToVkIndexType(EIndexType indexType)
{
	switch (indexType)
	{
	case EIndexType::INDEX_TYPE_UINT8:
		return (VK_INDEX_TYPE_UINT8);
	case EIndexType::INDEX_TYPE_UINT16:
		return (VK_INDEX_TYPE_UINT16);
	case EIndexType::INDEX_TYPE_UINT32:
		return (VK_INDEX_TYPE_UINT32);
	default:
		return (VK_INDEX_TYPE_UINT32);
	}
}

VkBufferUsageFlags VulkanUtils::ToVulkanBufferUsage(EBufferType type, bool cpuWritable)
{
	switch (type)
	{
	case EBufferType::BUFFER_TYPE_UNIFORM:
		return VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;

	case EBufferType::BUFFER_TYPE_STORAGE:
		return VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;

	case EBufferType::BUFFER_TYPE_INDEX:
		return cpuWritable ? (VK_BUFFER_USAGE_INDEX_BUFFER_BIT) : (VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);

	case EBufferType::BUFFER_TYPE_VERTEX:
		return cpuWritable ? (VK_BUFFER_USAGE_VERTEX_BUFFER_BIT) : (VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT);

	case EBufferType::BUFFER_TYPE_INDIRECT:
		return VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;

	case EBufferType::BUFFER_TYPE_TRANSFER_SRC:
		return VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

	case EBufferType::BUFFER_TYPE_TRANSFER_DST:
		return VK_BUFFER_USAGE_TRANSFER_DST_BIT;

	case EBufferType::BUFFER_TYPE_STAGING:
		return VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

	default:
		return VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
	}
}

VkMemoryPropertyFlags VulkanUtils::ToVulkanMemoryProperties(EBufferMemoryType type)
{
	switch (type)
	{
	case EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY:
		return VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT;

	case EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE:
		return VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT;

	case EBufferMemoryType::BUFFER_MEMORY_CPU_READ:
		return VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT;

	case EBufferMemoryType::BUFFER_MEMORY_CPU_READ_WRITE:
		return VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT | VK_MEMORY_PROPERTY_HOST_CACHED_BIT;
	}
	return VkMemoryPropertyFlags();
}

VkImageType VulkanUtils::ToVkImageType(ETextureType type)
{
	switch (type)
	{
	case ETextureType::TEXTURE_TYPE_1D:
		return VK_IMAGE_TYPE_1D;

	case ETextureType::TEXTURE_TYPE_2D:
		return VK_IMAGE_TYPE_2D;

	case ETextureType::TEXTURE_TYPE_3D:
		return VK_IMAGE_TYPE_3D;

	case ETextureType::TEXTURE_TYPE_CUBE:
		return VK_IMAGE_TYPE_2D; // cube uses 2D image type, cube view later

	default:
		return VK_IMAGE_TYPE_2D;
	}
}

VkImageViewType VulkanUtils::ToVkImageViewType(ETextureType type)
{
	switch (type)
	{
	case ETextureType::TEXTURE_TYPE_1D:
		return VK_IMAGE_VIEW_TYPE_1D;

	case ETextureType::TEXTURE_TYPE_2D:
		return VK_IMAGE_VIEW_TYPE_2D;

	case ETextureType::TEXTURE_TYPE_3D:
		return VK_IMAGE_VIEW_TYPE_3D;

	case ETextureType::TEXTURE_TYPE_CUBE:
		return VK_IMAGE_VIEW_TYPE_CUBE;

	default:
		return VK_IMAGE_VIEW_TYPE_2D;
	}
}

VkFormat VulkanUtils::ToVkFormat(ETextureFormats format)
{
	switch (format)
	{
	case ETextureFormats::TEXTIRE_FORMAT_RGBA8_UNORM:
		return VK_FORMAT_R8G8B8A8_UNORM;

	case ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB:
		return VK_FORMAT_R8G8B8A8_SRGB;

	case ETextureFormats::TEXTIRE_FORMAT_BGRA8_UNORM:
		return VK_FORMAT_B8G8R8A8_UNORM;

	case ETextureFormats::TEXTIRE_FORMAT_BGRA8_SRGB:
		return VK_FORMAT_B8G8R8A8_SRGB;

	case ETextureFormats::TEXTIRE_FORMAT_R8_UNORM:
		return VK_FORMAT_R8_UNORM;

	case ETextureFormats::TEXTIRE_FORMAT_RG8_UNORM:
		return VK_FORMAT_R8G8_UNORM;

	case ETextureFormats::TEXTIRE_FORMAT_RGBA16_FLOAT:
		return VK_FORMAT_R16G16B16A16_SFLOAT;

	case ETextureFormats::TEXTIRE_FORMAT_RGBA32_FLOAT:
		return VK_FORMAT_R32G32B32A32_SFLOAT;

	case ETextureFormats::TEXTIRE_FORMAT_DEPTH24_STENCIL8:
		return VK_FORMAT_D24_UNORM_S8_UINT;

	case ETextureFormats::TEXTIRE_FORMAT_DEPTH32F:
		return VK_FORMAT_D32_SFLOAT;

	default:
		return VK_FORMAT_R8G8B8A8_SRGB;
	}
}

VkImageUsageFlags VulkanUtils::ToVkImageUsage(uint32_t usageFlags)
{
	VkImageUsageFlags vkUsage = 0;

	if (usageFlags & static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_SAMPLED))
	{
		vkUsage |= VK_IMAGE_USAGE_SAMPLED_BIT;
	}

	if (usageFlags & static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_STORAGE))
	{
		vkUsage |= VK_IMAGE_USAGE_STORAGE_BIT;
	}

	if (usageFlags & static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_COLOR_ATTACHMENT))
	{
		vkUsage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
	}

	if (usageFlags & static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_DEPTH_STENCIL))
	{
		vkUsage |= VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	}

	if (usageFlags & static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_TRANSFER_SRC))
	{
		vkUsage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	}

	if (usageFlags & static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_TRANSFER_DST))
	{
		vkUsage |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
	}

	return vkUsage;
}

VkFilter VulkanUtils::ToVkFilter(ETextureFilter filter)
{
	switch (filter)
	{
	case ETextureFilter::TEXTURE_FILTER_NEAREST:
		return VK_FILTER_NEAREST;

	case ETextureFilter::TEXTURE_FILTER_LINEAR:
		return VK_FILTER_LINEAR;

	default:
		return VK_FILTER_LINEAR;
	}
}

VkSamplerMipmapMode VulkanUtils::ToVkMipmapMode(ETextureFilter filter)
{
	switch (filter)
	{
	case ETextureFilter::TEXTURE_FILTER_NEAREST:
		return VK_SAMPLER_MIPMAP_MODE_NEAREST;

	case ETextureFilter::TEXTURE_FILTER_LINEAR:
		return VK_SAMPLER_MIPMAP_MODE_LINEAR;

	default:
		return VK_SAMPLER_MIPMAP_MODE_LINEAR;
	}
}

VkSamplerAddressMode VulkanUtils::ToVkAddressMode(ETextureWrap wrap)
{
	switch (wrap)
	{
	case ETextureWrap::TEXTURE_WRAP_REPEAT:
		return VK_SAMPLER_ADDRESS_MODE_REPEAT;

	case ETextureWrap::TEXTURE_WRAP_MIRRORED_REPEAT:
		return VK_SAMPLER_ADDRESS_MODE_MIRRORED_REPEAT;

	case ETextureWrap::TEXTURE_WRAP_CLAMP_TO_EDGE:
		return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;

	case ETextureWrap::TEXTURE_WRAP_CLAMP_TO_BORDER:
		return VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_BORDER;

	default:
		return VK_SAMPLER_ADDRESS_MODE_REPEAT;
	}
}

VkDescriptorType VulkanUtils::ToVkDescriptorType(EBindingType type)
{
	switch (type)
	{

	case EBindingType::BIND_TYPE_SAMPLER:
		return VK_DESCRIPTOR_TYPE_SAMPLER;

	case EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER:
		return VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;

	case EBindingType::BIND_TYPE_SAMPLED_IMAGE:
		return VK_DESCRIPTOR_TYPE_SAMPLED_IMAGE;

	case EBindingType::BIND_TYPE_UNIFORM_BUFFER:
		return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;

	case EBindingType::BIND_TYPE_STORAGE_BUFFER:
		return VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;

	default:
		return VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
	}
}

VkImageLayout VulkanUtils::ToVkImageLayout(EImageBindingLayout imageBindingLayout)
{
	switch (imageBindingLayout)
	{
	case EImageBindingLayout::IMAGE_LAYOUT_UNDEFINED:
		return (VK_IMAGE_LAYOUT_UNDEFINED);
	case EImageBindingLayout::IMAGE_LAYOUT_GENERAL:
		return (VK_IMAGE_LAYOUT_GENERAL);
	case EImageBindingLayout::IMAGE_LAYOUT_COLOR_ATTACHMENT:
		return (VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL);
	case EImageBindingLayout::IMAGE_LAYOUT_SHADER_READ_ONLY:
		return (VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
	case EImageBindingLayout::IMAGE_LAYOUT_TRANSFER_SRC:
		return (VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL);
	case EImageBindingLayout::IMAGE_LAYOUT_TRANSFER_DST:
		return (VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
	case EImageBindingLayout::IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT:
		return (VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);
	case EImageBindingLayout::IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY:
		return (VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL);

	default:
		syserr("Unknown Image Layout {}", static_cast<uint32_t>(imageBindingLayout));
		return (VK_IMAGE_LAYOUT_UNDEFINED);
	}
}

VkShaderStageFlagBits VulkanUtils::ToVkShaderInfo(EShaderStage shaderType)
{
	switch (shaderType)
	{
	case EShaderStage::SHADER_TYPE_VERTEX:
		return (VK_SHADER_STAGE_VERTEX_BIT);

	case EShaderStage::SHADER_TYPE_TESSELLATION_CONTROL:
		return (VK_SHADER_STAGE_TESSELLATION_CONTROL_BIT);

	case EShaderStage::SHADER_TYPE_TESSELLATION_EVALUATION:
		return (VK_SHADER_STAGE_TESSELLATION_EVALUATION_BIT);

	case EShaderStage::SHADER_TYPE_GEOMETRY:
		return (VK_SHADER_STAGE_GEOMETRY_BIT);

	case EShaderStage::SHADER_TYPE_FRAGMENT:
		return (VK_SHADER_STAGE_FRAGMENT_BIT);

	case EShaderStage::SHADER_TYPE_COMPUTE:
		return (VK_SHADER_STAGE_COMPUTE_BIT);

	default:
		return (VK_SHADER_STAGE_VERTEX_BIT);
	}
}
