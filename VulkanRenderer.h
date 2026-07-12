#pragma once

#include "VulkanDevice.h"
#include "VulkanPipeline.h"
#include "VulkanBuffer.h"
#include "VulkanSyncObject.h"
#include "VulkanUtils.h"
#include "VulkanDescriptorContext.h"
#include "VulkanTexture.h"
#include "VulkanModel/StaticModel.h"
#include "TexturesManager.h"
#include "VulkanSwapchain.h"

class CVulkanRenderer
{
public:
	CVulkanRenderer() = default;
	~CVulkanRenderer() = default;

	bool InitializeRenderer();
	void Clear();
	void Destroy();

	bool RecreateSwapchainResources();

	void RecordCommandBuffer(VkCommandBuffer commandBuffer, uint32_t imageIndex);
	void DrawFrame();
	void SetFrameBufferResized(bool bValue);

	void UpdateUniformBuffers(uint32_t currFrame);
	bool CreateFallbackTextures();
	void ClearFallBackTextures();

protected:
	CVulkanDevice vulkan_device;

private:
	CVulkanSwapchain* m_pVulkanSwapchain = nullptr;
	CVulkanPipeline* m_pVulkanStaticMeshPipeline = nullptr;
	CVulkanSyncObject* m_pVulkanSyncObject = nullptr;
	CVulkanPatch* m_pVulkanPatch = nullptr;
	CTexturesManager* m_pTexturesManager = nullptr;
	std::shared_ptr<CStaticModel> m_pStaticModel1 = nullptr;
	std::shared_ptr<CStaticModel> m_pStaticModel2 = nullptr;

	std::vector<VkFramebuffer> m_vkvSwapchainFrameBuffers = {};
	std::vector<VkCommandBuffer> m_vkvCommandBuffers = {};

	VkImage m_vkDepthImage = VK_NULL_HANDLE;
	VkDeviceMemory m_vkDepthImageMemory = VK_NULL_HANDLE;
	VkImageView m_vkDepthImageView = VK_NULL_HANDLE;
	VkFormat m_vkDepthFormat = VK_FORMAT_UNDEFINED;

	// Semaphores
	uint32_t m_uiCurrentFrame = 0;
	bool m_bFramebufferResized = false;

	// VertexBuffer
	std::vector<CVulkanBuffer*> m_vpUniformBuffer = {};
};