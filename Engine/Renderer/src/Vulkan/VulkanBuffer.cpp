#include "Vulkan/VulkanBuffer.h"

void CVulkanBuffer::Clear()
{
	m_vkBuffer = VK_NULL_HANDLE;
	m_vkBufferMemory = VK_NULL_HANDLE;

	// Buffer Properties
	m_stName = "Buffer";
	m_eType = EBufferType::BUFFER_TYPE_VERTEX;

	m_uiSize = 0;
	m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_GPU_ONLY;
	m_bIsValid = false;
}

void CVulkanBuffer::UpdateBufferData(const SVulkanBufferData& vulkanBufferData, const SBufferDesc& bufferDesc)
{
	m_vkBuffer = vulkanBufferData.m_vkBuffer;
	m_vkBufferMemory = vulkanBufferData.m_vkBufferMemory;
	m_vkUsageFlags = vulkanBufferData.m_vkUsageFlags;
	m_vkMemoryPropertyFlags = vulkanBufferData.m_vkMemoryPropertyFlags;
	m_vkSharingMode = vulkanBufferData.m_vkSharingMode;

	m_stName = bufferDesc.m_stName;
	m_eType = bufferDesc.m_eType;
	m_eMemoryType = bufferDesc.m_eMemoryType;
	m_uiSize = bufferDesc.m_uiSize;
	m_bIsValid = true;
}
