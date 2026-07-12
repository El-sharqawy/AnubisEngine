#pragma once

#include "API/Texture.h"

struct SVulkanTexture2DInfo
{
    VkImage m_vkImage = VK_NULL_HANDLE;
    VkDeviceMemory m_vkImageMemory = VK_NULL_HANDLE;
    VkImageView m_vkImageView = VK_NULL_HANDLE;
    VkSampler m_vkSampler = VK_NULL_HANDLE;
    int32_t iWidth;
    int32_t iHeight;
    int32_t iChannels;
};

class CVulkanTexture2D : public ITexture2D
{
public:
    CVulkanTexture2D() = default;
	~CVulkanTexture2D() = default;

	VkImage GetImage() const;
	VkDeviceMemory GetImageMemory() const;
	VkImageView GetImageView() const;
	VkSampler GetSampler() const;

	void Clear();
	void UpdateTextureData(const SVulkanTexture2DInfo& textureInfo, const STextureDesc& textureDesc);

    uint32_t GetWidth() const override;
    uint32_t GetHeight() const override;
    std::string GetName() const;

	// Vulkan Specified
private:
	VkImage m_vkImage = VK_NULL_HANDLE;
	VkDeviceMemory m_vkImageMemory = VK_NULL_HANDLE;
	VkImageView m_vkImageView = VK_NULL_HANDLE;
	VkSampler m_vkSampler = VK_NULL_HANDLE;
    SVulkanTextureDesc		m_vkTextureDesc = {};
};