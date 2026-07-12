#pragma once

#include "Vulkan/VulkanUtils.h"
#include <optional>
#include <vector>

struct SQueueFamilyIndices
{
	std::optional<uint32_t> graphicsQueueFamily;
	std::optional<uint32_t> presentQueueFamily;

	bool IsComplete() const
	{
		return graphicsQueueFamily.has_value() && presentQueueFamily.has_value();
	}
};

struct SSwapChainSupportDetails
{
	VkSurfaceCapabilitiesKHR capabilities;
	std::vector<VkSurfaceFormatKHR> formats;
	std::vector<VkPresentModeKHR> presentModes;
};


class CVulkanDevice
{
public:
	CVulkanDevice() = default;
	~CVulkanDevice() = default;

	bool Initialize(); // connect it to glfw
	bool InitializeSurface(GLFWwindow* pPlatformWindowHandle);
	bool ChoosePhysicalDevice();
	bool CreateLogicalDevice();
	bool CreateCommandPool();
	uint32_t FindMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties) const;
	VkFormat FindSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features);
	VkFormat FindDepthFormat();
	bool FormatHasStencilComponent(VkFormat format);

	void Destroy();

	const std::vector<const char*> GetExtensions() const;
	bool IsDeviceSuitable(VkPhysicalDevice device);
	SQueueFamilyIndices FindFamilyIndices(VkPhysicalDevice device);
	SSwapChainSupportDetails QuerySwapChainSupport();

protected:
	static VKAPI_ATTR VkBool32 VKAPI_CALL DebuggingCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageTypes, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData);

public:
	VkInstance GetInstance() const;
	VkSurfaceKHR GetDeviceSurface() const;
	VkPhysicalDevice GetPhysicalDevice() const;
	VkDevice GetDevice() const;
	VkQueue GetGraphicsQueue() const;
	VkQueue GetPresentQueue() const;
	uint32_t GetGraphicsQueueFamily();
	uint32_t GetPresentQueueFamily();
	VkCommandPool GetCommandPool() const;

	VkCommandBuffer BeginSingleTimeCommands();
	bool EndSingleTimeCommands(VkCommandBuffer cmd);

private:
	VkInstance m_vkInstance = VK_NULL_HANDLE;
	VkDebugUtilsMessengerEXT m_vkDebugMessenger = VK_NULL_HANDLE;
	VkSurfaceKHR m_vkDeviceSurface = VK_NULL_HANDLE;
	VkPhysicalDevice m_vkPhysicalDevice = VK_NULL_HANDLE;
	VkDevice m_vkDevice = VK_NULL_HANDLE;
	VkQueue m_vkGraphicsQueue = VK_NULL_HANDLE;
	VkQueue m_vkPresentQueue = VK_NULL_HANDLE;
	VkCommandPool m_vkCommandPool = VK_NULL_HANDLE;
	std::vector<VkCommandBuffer> m_vkvCommandBuffers = {};
	SQueueFamilyIndices m_gQueueFamilyIndices = {};
};