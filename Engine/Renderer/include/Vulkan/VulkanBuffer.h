#pragma once

#include "API/Buffer.h"
#include <vulkan/vulkan.h>
#include <string>

struct SVulkanBufferData
{
	VkBuffer m_vkBuffer = VK_NULL_HANDLE;
	VkDeviceMemory m_vkBufferMemory = VK_NULL_HANDLE;
	VkBufferUsageFlags m_vkUsageFlags = 0;
	VkMemoryPropertyFlags m_vkMemoryPropertyFlags = 0;
	VkSharingMode m_vkSharingMode = VK_SHARING_MODE_EXCLUSIVE;
};

class CVulkanBuffer : public IBuffer
{
public:
	CVulkanBuffer() = default;
	~CVulkanBuffer() = default;

	// Non-copyable — owns GPU resources
	CVulkanBuffer(const CVulkanBuffer&) = delete;
	CVulkanBuffer& operator=(const CVulkanBuffer&) = delete;

	void Clear();
	void UpdateBufferData(const SVulkanBufferData& vulkanBufferData, const SBufferDesc& bufferDesc);

	// Vulkan Only Data
	VkBuffer GetBuffer() const { return (m_vkBuffer); }
	VkDeviceMemory GetBufferMemory() const { return (m_vkBufferMemory); }
	VkBufferUsageFlags GetUsageFlags() const { return (m_vkUsageFlags); }
	VkMemoryPropertyFlags GetMemoryPropertyFlags() const { return (m_vkMemoryPropertyFlags); }
	VkSharingMode GetSharingMode() const { return (m_vkSharingMode); }

	// Buffer Properties
	const std::string& GetName() const override { return (m_stName); }
	EBufferType GetType() const override { return (m_eType); }
	EBufferMemoryType GetMemoryType() const override { return (m_eMemoryType); }
	uint64_t GetSize() const override { return (m_uiSize); }
	bool IsValid() const override { return (m_bIsValid); }


private:
	// Vulkan Properties
	VkBuffer m_vkBuffer = VK_NULL_HANDLE;
	VkDeviceMemory m_vkBufferMemory = VK_NULL_HANDLE;
	VkBufferUsageFlags m_vkUsageFlags = 0;
	VkMemoryPropertyFlags m_vkMemoryPropertyFlags = 0;
	VkSharingMode m_vkSharingMode = VK_SHARING_MODE_EXCLUSIVE;
};