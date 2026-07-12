#pragma once

#include <vector>
#include <string>
#include <filesystem>
#include <cstdint>
#include <vulkan/vulkan.h>
#include <stb_image/stb_image.h>

enum class ETextureFormat
{
	Unknown = 0,
	R8,
	RG8,
	RGB8,
	RGBA8,
	BGRA8,

	SRGB8,
	SRGB8_ALPHA8
};

struct STextureDesc
{
	std::string				m_stName = "Texture";
	std::filesystem::path	m_fsFilePath;           // empty = procedural texture
	int32_t                 m_iWidth = 0;
	int32_t                 m_iHeight = 0;
	int32_t					m_iChannels = 0;
	VkImageType				m_eTexTarget = VK_IMAGE_TYPE_2D;
	VkFormat                m_eSourceDataType = VK_FORMAT_R8G8B8A8_SRGB;
	VkFilter				m_eMinFilter = VK_FILTER_LINEAR;
	VkFilter				m_eMagFilter = VK_FILTER_LINEAR;
	VkSamplerAddressMode	m_eWrapU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	VkSamplerAddressMode	m_eWrapV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	VkSamplerAddressMode	m_eWrapW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
	bool					m_bGenMipmaps = true;
	bool					m_bBindless = false;

	// For procedural/colored textures — null means load from file
	void* m_pData = nullptr;
};

struct SImageData
{
	std::string stName;				// Path to the image file
	int32_t iWidth;					// Width of the image
	int32_t iHeight;				// Height of the image
	int32_t iChannels;				// Number of color channels (e.g., 3 for RGB, 4 for RGBA)
	uint8_t* pData;					// Pointer to the image data

	// Helper to check if loading worked
	bool IsValid() const
	{
		return pData != nullptr;
	}
};

struct SDecodedImage
{
	std::filesystem::path m_fsSourcePath;
	std::string m_stName;

	int32_t m_iWidth = 0;
	int32_t m_iHeight = 0;
	int32_t m_iComponentCount = 0;

	ETextureFormat m_eFormat = ETextureFormat::Unknown;

	std::vector<uint8_t> m_vPixels;

	bool IsValid() const noexcept
	{
		return m_iWidth > 0 &&
			m_iHeight > 0 &&
			!m_vPixels.empty() &&
			m_eFormat != ETextureFormat::Unknown;
	}

	void Clear()
	{
		m_fsSourcePath.clear();
		m_stName.clear();
		m_iWidth = 0;
		m_iHeight = 0;
		m_iComponentCount = 0;
		m_eFormat = ETextureFormat::Unknown;
		m_vPixels.clear();
		m_vPixels.shrink_to_fit();
	}

	size_t GetDataSize() const noexcept
	{
		return m_vPixels.size();
	}

	const uint8_t* GetData() const noexcept
	{
		return m_vPixels.empty() ? nullptr : m_vPixels.data();
	}

	uint8_t* GetData() noexcept
	{
		return m_vPixels.empty() ? nullptr : m_vPixels.data();
	}
};

static inline bool IsValidImage(const std::string& filePath)
{
	// Check file existence and regular file status
	if (!std::filesystem::exists(filePath) || !std::filesystem::is_regular_file(filePath))
	{
		return (false);
	}

	int32_t width, height, channels;
	const bool isValid = stbi_info(filePath.c_str(), &width, &height, &channels);

	return (isValid);
}
