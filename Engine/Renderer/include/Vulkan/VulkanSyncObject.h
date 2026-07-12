#pragma once

#include <vulkan/vulkan.h>
#include <vector>

class CVulkanDevice;

class CVulkanSyncObject
{
public:
	CVulkanSyncObject() = default;
	~CVulkanSyncObject() = default;

	bool Initialize(size_t swapChainImages);
	void Destroy();

	void WaitForFrameFence(uint32_t frameIndex);
	void ResetFrameFence(uint32_t frameIndex);
	VkResult AcquireNextImage(VkSwapchainKHR swapChain, uint64_t timeout, uint32_t frameIndex, uint32_t& imageIndex);
	VkSemaphore GetImageAvailableSemaphore(uint32_t frameIndex);
	VkSemaphore GetRenderFinishedSemaphore(uint32_t frameIndex);
	VkFence GetFlightFence(uint32_t frameIndex);

private:
	std::vector<VkSemaphore> m_vkvImagesAvailableSemaphores = {};
	std::vector<VkSemaphore> m_vkvRenderingFinishedSemaphores = {};
	std::vector<VkFence> m_vkvFlightFences = {};
	size_t m_uiSwapchainImageCount = 0;
};