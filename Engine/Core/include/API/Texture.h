#pragma once

#include <vulkan/vulkan.h>
#include <string>
#include <filesystem>

enum class ETextureType
{
    TEXTURE_TYPE_1D,
    TEXTURE_TYPE_2D,
    TEXTURE_TYPE_3D,
    TEXTURE_TYPE_CUBE,
};

enum class ETextureFormats
{
    TEXTIRE_FORMAT_R8_UNORM,
    TEXTIRE_FORMAT_RG8_UNORM,
    TEXTIRE_FORMAT_RGB8_UNORM,
    TEXTIRE_FORMAT_RGBA8_UNORM,
    TEXTIRE_FORMAT_RGBA8_SRGB,
    TEXTIRE_FORMAT_BGRA8_UNORM,
    TEXTIRE_FORMAT_BGRA8_SRGB,

    TEXTIRE_FORMAT_R16_FLOAT,
    TEXTIRE_FORMAT_R32_FLOAT,
    TEXTIRE_FORMAT_RG16_FLOAT,
    TEXTIRE_FORMAT_RG32_FLOAT,
    TEXTIRE_FORMAT_RGBA16_FLOAT,
    TEXTIRE_FORMAT_RGBA32_FLOAT,

    TEXTIRE_FORMAT_DEPTH16,
    TEXTIRE_FORMAT_DEPTH24_STENCIL8,
    TEXTIRE_FORMAT_DEPTH32F,
    TEXTIRE_FORMAT_DEPTH32F_STENCIL8,
};

enum class ETextureFilter
{
    TEXTURE_FILTER_NEAREST,
    TEXTURE_FILTER_LINEAR
};

enum class ETextureMipmapMode
{
    TEXTURE_MIPMAP_MODE_NONE,
    TEXTURE_MIPMAP_MODE_NEAREST,
    TEXTURE_MIPMAP_MODE_LINEAR
};

enum class ETextureWrap
{
    TEXTURE_WRAP_REPEAT,
    TEXTURE_WRAP_CLAMP_TO_EDGE,
    TEXTURE_WRAP_CLAMP_TO_BORDER,
    TEXTURE_WRAP_MIRRORED_REPEAT,
    TEXTURE_WRAP_MIRROR_CLAMP_TO_EDGE,
};

enum class ETextureBorderColor
{
    TEXTURE_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK,
    TEXTURE_BORDER_COLOR_FLOAT_OPAQUE_BLACK,
    TEXTURE_BORDER_COLOR_FLOAT_OPAQUE_WHITE,
};

enum class ETextureUsage : uint32_t
{
    TEXTURE_USAGE_SAMPLED = 1 << 0,
    TEXTURE_USAGE_COLOR_ATTACHMENT = 1 << 1,
    TEXTURE_USAGE_DEPTH_STENCIL = 1 << 2,
    TEXTURE_USAGE_TRANSFER_DST = 1 << 3,
    TEXTURE_USAGE_TRANSFER_SRC = 1 << 4,
    TEXTURE_USAGE_STORAGE = 1 << 5,
};

struct SVulkanTextureDesc
{
    VkImageTiling            m_vkImageTiling = VK_IMAGE_TILING_OPTIMAL;
    VkImageLayout            m_vkInitialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    VkSampleCountFlagBits    m_vkSamples = VK_SAMPLE_COUNT_1_BIT;
    VkSharingMode            m_vkSharingMode = VK_SHARING_MODE_EXCLUSIVE;
};

struct STextureDesc
{
    std::string				m_stName = "Texture";
    std::filesystem::path	m_fsFilePath;           // empty = procedural texture
    int32_t					m_iWidth = 0;
    int32_t					m_iHeight = 0;
    int32_t					m_iDepth = 1;
    int32_t					m_iChannels = 0;
    ETextureType            m_eType = ETextureType::TEXTURE_TYPE_2D;
    ETextureFormats         m_eFormat = ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB;
    uint32_t                m_uiUsageFlags = static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_SAMPLED) | static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_TRANSFER_DST);
    ETextureFilter          m_eMagFilter = ETextureFilter::TEXTURE_FILTER_LINEAR;
    ETextureFilter          m_eMinFilter = ETextureFilter::TEXTURE_FILTER_LINEAR;
    ETextureMipmapMode      m_eMipmapMode = ETextureMipmapMode::TEXTURE_MIPMAP_MODE_LINEAR;

    ETextureWrap            m_eWrapU = ETextureWrap::TEXTURE_WRAP_REPEAT;
    ETextureWrap            m_eWrapV = ETextureWrap::TEXTURE_WRAP_REPEAT;
    ETextureWrap            m_eWrapW = ETextureWrap::TEXTURE_WRAP_REPEAT;
    bool					m_bGenMipmaps = true;
    bool                    m_bEnableAnisotropy = true;
    float                   m_fMaxAnisotropy = 16.0f;
    bool                    m_bEnableCompare = false; // useful for shadow maps later

    SVulkanTextureDesc		m_vkTextureDesc = {};
};

class ITexture2D
{
public:
    virtual ~ITexture2D() = default;
    virtual uint32_t GetWidth() const = 0;
    virtual uint32_t GetHeight() const = 0;

protected:
    // Texture Properties
    std::string				m_stName = "Texture";
    std::filesystem::path	m_fsFilePath;           // empty = procedural texture
    int32_t					m_iWidth = 0;
    int32_t					m_iHeight = 0;
    int32_t					m_iDepth = 1;
    int32_t					m_iChannels = 0;
    ETextureType            m_eType = ETextureType::TEXTURE_TYPE_2D;
    ETextureFormats         m_eFormat = ETextureFormats::TEXTIRE_FORMAT_RGBA8_SRGB;
    uint32_t                m_uiUsageFlags = static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_SAMPLED) | static_cast<uint32_t>(ETextureUsage::TEXTURE_USAGE_TRANSFER_DST);
    ETextureFilter          m_eMagFilter = ETextureFilter::TEXTURE_FILTER_LINEAR;
    ETextureFilter          m_eMinFilter = ETextureFilter::TEXTURE_FILTER_LINEAR;
    ETextureMipmapMode      m_eMipmapMode = ETextureMipmapMode::TEXTURE_MIPMAP_MODE_LINEAR;

    ETextureWrap            m_eWrapU = ETextureWrap::TEXTURE_WRAP_REPEAT;
    ETextureWrap            m_eWrapV = ETextureWrap::TEXTURE_WRAP_REPEAT;
    ETextureWrap            m_eWrapW = ETextureWrap::TEXTURE_WRAP_REPEAT;
    bool					m_bGenMipmaps = false;
    bool                    m_bEnableAnisotropy = true;
    float                   m_fMaxAnisotropy = 16.0f;
    bool                    m_bEnableCompare = false; // useful for shadow maps later
};
