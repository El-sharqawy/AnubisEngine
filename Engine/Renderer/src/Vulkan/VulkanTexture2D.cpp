#include "Vulkan/VulkanTexture2D.h"
#include "Logging/LogManager.h"

VkImage CVulkanTexture2D::GetImage() const
{
    return (m_vkImage);
}

VkDeviceMemory CVulkanTexture2D::GetImageMemory() const
{
    return (m_vkImageMemory);
}

VkImageView CVulkanTexture2D::GetImageView() const
{
    return (m_vkImageView);
}

VkSampler CVulkanTexture2D::GetSampler() const
{
    return (m_vkSampler);
}

void CVulkanTexture2D::Clear()
{
    m_vkImage = VK_NULL_HANDLE;
    m_vkImageMemory = VK_NULL_HANDLE;
    m_vkImageView = VK_NULL_HANDLE;
    m_vkSampler = VK_NULL_HANDLE;

    m_stName = "Texture";
    m_fsFilePath.clear();
    m_iWidth = 0;
    m_iHeight = 0;
    m_iDepth = 1;
    m_iChannels = 0;
    m_eType = ETextureType::TEXTURE_TYPE_2D;
    m_eFormat = ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB;
    m_uiUsageFlags = static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_SAMPLED) | static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_TRANSFER_DST);
    m_eMagFilter = ETextureFilter::TEXTURE_FILTER_LINEAR;
    m_eMinFilter = ETextureFilter::TEXTURE_FILTER_LINEAR;
    m_eMipmapMode = ETextureMipmapMode::TEXTURE_MIPMAP_MODE_LINEAR;

    m_eWrapU = ETextureWrap::TEXTURE_WRAP_REPEAT;
    m_eWrapV = ETextureWrap::TEXTURE_WRAP_REPEAT;
    m_eWrapW = ETextureWrap::TEXTURE_WRAP_REPEAT;
    m_bGenMipmaps = true;
    m_bEnableAnisotropy = true;
    m_fMaxAnisotropy = 16.0f;
    m_bEnableCompare = false;
    m_bIsValid = false;

    m_vkTextureDesc = {};
}

void CVulkanTexture2D::UpdateTextureData(const SVulkanTexture2DInfo& textureInfo, const STextureDesc& textureDesc)
{
    m_vkImage = textureInfo.m_vkImage;
    m_vkImageMemory = textureInfo.m_vkImageMemory;
    m_vkImageView = textureInfo.m_vkImageView;
    m_vkSampler = textureInfo.m_vkSampler;

    if (textureInfo.m_vkImage == VK_NULL_HANDLE ||
        textureInfo.m_vkImageView == VK_NULL_HANDLE ||
        textureInfo.m_vkSampler == VK_NULL_HANDLE)
    {
        syserr("Invalid texture handles in UpdateTextureData");
    }

    m_stName = textureDesc.m_stName;
    // empty = procedural texture
    m_fsFilePath = textureDesc.m_fsFilePath;
    m_iWidth = textureInfo.iWidth;
    m_iHeight = textureInfo.iHeight;
    m_iDepth = 1;			// depth is mostly 1 unless 3D
    m_iChannels = textureInfo.iChannels;
    m_eType = textureDesc.m_eType;
    m_eFormat = textureDesc.m_eFormat;
    m_uiUsageFlags = textureDesc.m_uiUsageFlags;
    m_eMagFilter = textureDesc.m_eMagFilter;
    m_eMinFilter = textureDesc.m_eMinFilter;
    m_eMipmapMode = textureDesc.m_eMipmapMode;

    m_eWrapU = textureDesc.m_eWrapU;
    m_eWrapV = textureDesc.m_eWrapV;
    m_eWrapW = textureDesc.m_eWrapW;
    m_bGenMipmaps = textureDesc.m_bGenMipmaps;
    m_bEnableAnisotropy = textureDesc.m_bEnableAnisotropy;
    m_fMaxAnisotropy = textureDesc.m_fMaxAnisotropy;
    m_bEnableCompare = textureDesc.m_bEnableCompare;
    m_bIsValid = textureDesc.m_bIsValid;
}

uint32_t CVulkanTexture2D::GetWidth() const
{
    return (m_iWidth);
}

uint32_t CVulkanTexture2D::GetHeight() const
{
    return (m_iHeight);
}

std::string CVulkanTexture2D::GetName() const
{
    return (m_stName);
}

bool CVulkanTexture2D::IsValid() const
{
    return (m_bIsValid);
}
