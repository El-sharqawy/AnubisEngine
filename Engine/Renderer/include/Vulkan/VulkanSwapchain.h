#pragma once

#include <vulkan/vulkan.h>
#include <vector>

class CVulkanSwapchain
{
public:
	CVulkanSwapchain() = default;
	~CVulkanSwapchain() = default;

	bool InitializeSwapchain();
	bool CreateImageViews();
	bool CreateDepthResources();
	bool CreateRenderPass();
	void DestroyDepthResources();

	bool CreateFramebuffers();
	void DestroyFramebuffers();

	void Clear();
	void Destroy();

	VkSwapchainKHR GetSwapchain() const;
	VkFormat GetSwapchainImageFormat() const;
	VkExtent2D GetSwapchainExtent() const;
	const std::vector<VkImage>& GetSwapchainImages() const;
	const std::vector<VkImageView>& GetSwapchainImageViews() const;
	const std::vector<VkFramebuffer>& GetFramebuffers() const;
	VkImage GetDepthImage() const;
	VkDeviceMemory GetDepthImageMemory() const;
	VkImageView GetDepthImageView() const;
	VkRenderPass GetRenderPass() const;
	VkFramebuffer GetFramebuffer(uint32_t imageIndex) const;

protected:
	VkSurfaceFormatKHR ChooseSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& formats);
	VkPresentModeKHR ChoosePresentMode(const std::vector<VkPresentModeKHR>& presentModes);
	VkExtent2D ChooseExtent(const VkSurfaceCapabilitiesKHR& capabilities);

private:
	VkSwapchainKHR m_vkSwapchain = VK_NULL_HANDLE;
	VkFormat m_vkSwapchainImageFormat = VK_FORMAT_UNDEFINED;
	VkExtent2D m_vkSwapchainExtent = { 0, 0 };
	std::vector<VkImage> m_vkvSwapchainImages = {};
	std::vector<VkImageView> m_vkvSwapChainImageViews = {};
	std::vector<VkFramebuffer> m_vkvSwapchainFrameBuffers = {};

	VkImage m_vkDepthImage = VK_NULL_HANDLE;
	VkDeviceMemory m_vkDepthImageMemory = VK_NULL_HANDLE;
	VkImageView m_vkDepthImageView = VK_NULL_HANDLE;

	VkRenderPass m_vkRenderPass = VK_NULL_HANDLE;


};