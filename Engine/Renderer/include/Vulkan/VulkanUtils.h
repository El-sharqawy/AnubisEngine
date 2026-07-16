#pragma once

#include <vulkan/vulkan.h>
#include <vulkan/vk_enum_string_helper.h>

#include <vector>
#include <fstream>
#include <array>
#include <cassert>

#include "TypeMatrix4.h"
#include "Logging/LogManager.h"
#include "API/Buffer.h"
#include "API/Texture.h"
#include "API/BindingContext.h"
#include "API/ShaderProgram.h"

#define VK_CHECK(call)                                           \
{                                                                \
    VkResult result = (call);                                    \
    if (result != VK_SUCCESS)                                    \
    {                                                            \
        printf("VkResult Error: %s in %s at line %d\n",         \
            string_VkResult(result), __FILE__, __LINE__);        \
        assert(result == VK_SUCCESS);                            \
    }                                                            \
}

#define VK_CHECK_BOOL(call) VulkanUtils::VulkanFailed((call), #call, __FILE__, __LINE__)

namespace VulkanUtils
{
	bool VulkanFailed(VkResult result, const char* expr, const char* file, int line);
	std::vector<uint32_t> ReadShaderData(const std::string& filename);
    VkPrimitiveTopology ToVkTopology(EPrimitiveTopology topology);
    VkCullModeFlags ToVkCullMode(ECullMode mode);
    VkFrontFace ToVkFrontFace(EFrontFace face);
    VkPolygonMode ToVkPolygonMode(EPolygonMode polygonMode);
    VkCompareOp ToVkCompareOp(EDepthCompareOp op);
    VkIndexType ToVkIndexType(EIndexType indexType);
    VkBufferUsageFlags ToVulkanBufferUsage(EBufferType type, bool cpuWritable);
    VkMemoryPropertyFlags ToVulkanMemoryProperties(EBufferMemoryType type);

    // Image
    VkImageType ToVkImageType(ETextureType type);
    VkImageViewType ToVkImageViewType(ETextureType type);
    VkFormat ToVkFormat(ETextureFormats format);
    VkImageUsageFlags ToVkImageUsage(uint32_t usageFlags);
    VkFilter ToVkFilter(ETextureFilter filter);
    VkSamplerMipmapMode ToVkMipmapMode(ETextureFilter filter);
    VkSamplerAddressMode ToVkAddressMode(ETextureWrap wrap);

    // Descriptor
    VkDescriptorType ToVkDescriptorType(EBindingType type);
    VkImageLayout ToVkImageLayout(EImageBindingLayout shaderType);

    // Shaders
    VkShaderStageFlagBits ToVkShaderInfo(EShaderStage shaderType);
}
