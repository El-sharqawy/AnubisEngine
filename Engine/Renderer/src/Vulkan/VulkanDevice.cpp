#include "Vulkan/VulkanDevice.h"
#include "Window/WindowManager.h"
#include <set>
#include <stb_image/stb_image.h>

bool CVulkanDevice::Initialize()
{
	VkApplicationInfo appInfo{};
	appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
	appInfo.pNext = nullptr;
	appInfo.pApplicationName = "Anubis Engine";
	appInfo.applicationVersion = VK_MAKE_VERSION(1, 0, 0);
	appInfo.pEngineName = "Anubis Engine";
	appInfo.engineVersion = VK_MAKE_VERSION(1, 0, 0);
	// appInfo.apiVersion = VK_MAKE_API_VERSION(0, 1, 4, 0);
	appInfo.apiVersion = VK_API_VERSION_1_4;

	VkInstanceCreateInfo instanceCreateInfo{};
	instanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
	instanceCreateInfo.flags = 0;
	instanceCreateInfo.pApplicationInfo = &appInfo;
	const std::vector<const char*> validationLayers = {
		"VK_LAYER_KHRONOS_validation"
	};

	instanceCreateInfo.enabledLayerCount = static_cast<uint32_t>(validationLayers.size());
	instanceCreateInfo.ppEnabledLayerNames = validationLayers.data();

	auto extensions = GetExtensions();
	instanceCreateInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
	instanceCreateInfo.ppEnabledExtensionNames = extensions.data();

	VkDebugUtilsMessengerCreateInfoEXT debugUtilsCreateInfo{};
	debugUtilsCreateInfo.sType = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
	debugUtilsCreateInfo.pNext = nullptr;
	debugUtilsCreateInfo.flags = 0;
	debugUtilsCreateInfo.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
	debugUtilsCreateInfo.messageType = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
	debugUtilsCreateInfo.pfnUserCallback = DebuggingCallback;
	debugUtilsCreateInfo.pUserData = nullptr;

	instanceCreateInfo.pNext = &debugUtilsCreateInfo;

	if (vkCreateInstance(&instanceCreateInfo, nullptr, &m_vkInstance) != VK_SUCCESS)
	{
		syserr("Failed to Create Vulkan Instance");
		return (false);
	}

	// Look up the address of the extension function dynamically
	// We can't call the function directly since it's extension
	auto funcCreate = (PFN_vkCreateDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_vkInstance, "vkCreateDebugUtilsMessengerEXT");
	if (funcCreate)
	{
		//  Call the FUNCTION POINTER (funcCreate), NOT the bare Vulkan name
		if (funcCreate(m_vkInstance, &debugUtilsCreateInfo, nullptr, &m_vkDebugMessenger) != VK_SUCCESS)
		{
			syserr("Failed to set up Vulkan standalone debug messenger!");
			return (false);
		}
	}
	else
	{
		syserr("Vulkan: Debug extension function not found.");
		return (false);
	}

	return (true);
}

bool CVulkanDevice::InitializeSurface(GLFWwindow* pPlatformWindowHandle)
{
	if (glfwCreateWindowSurface(m_vkInstance, pPlatformWindowHandle, nullptr, &m_vkDeviceSurface) != VK_SUCCESS)
	{
		syserr("Failed to create GLFW Vulkan surface");
		return false;
	}

	return (true);

	/*
	VkWin32SurfaceCreateInfoKHR win32SurfaceCreateInfo{};
	win32SurfaceCreateInfo.sType = VK_STRUCTURE_TYPE_WIN32_SURFACE_CREATE_INFO_KHR;
	win32SurfaceCreateInfo.pNext = nullptr;
	win32SurfaceCreateInfo.flags = 0; // reserved
	win32SurfaceCreateInfo.hinstance = GetModuleHandle(nullptr);

	win32SurfaceCreateInfo.hwnd = reinterpret_cast<HWND>(pWindowNativeHandle);

	if (vkCreateWin32SurfaceKHR(m_vkInstance, &win32SurfaceCreateInfo, nullptr, &m_vkDeviceSurface) != VK_SUCCESS)
	{
		syserr("Failed to Create Win32 Surface KHR");
		return (false);
	}*/
}

bool CVulkanDevice::ChoosePhysicalDevice()
{
	uint32_t uiPhysicalDevicesNum = 0;
	vkEnumeratePhysicalDevices(m_vkInstance, &uiPhysicalDevicesNum, nullptr);

	if (uiPhysicalDevicesNum == 0)
	{
		syserr("Failed to Obtain Physical Devices");
		return (false);
	}

	std::vector<VkPhysicalDevice> vPhysicalDevices(uiPhysicalDevicesNum);
	vkEnumeratePhysicalDevices(m_vkInstance, &uiPhysicalDevicesNum, vPhysicalDevices.data());

	for (auto& physicalDevice : vPhysicalDevices)
	{
		if (IsDeviceSuitable(physicalDevice))
		{
			m_vkPhysicalDevice = physicalDevice;
			break;
		}
	}

	if (m_vkPhysicalDevice == VK_NULL_HANDLE)
	{
		syserr("No suitable physical devices found!");
		return false;
	}

	return (true);
}

bool CVulkanDevice::CreateLogicalDevice()
{
	SQueueFamilyIndices indices = FindFamilyIndices(m_vkPhysicalDevice);
	std::set<uint32_t> familyIndices = { indices.graphicsQueueFamily.value(), indices.presentQueueFamily.value() };

	std::vector<VkDeviceQueueCreateInfo> queuesInfo;

	float fQueuePriority = 1.0f;
	for (auto& index : familyIndices)
	{
		VkDeviceQueueCreateInfo queueCreationInfo{};
		queueCreationInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
		queueCreationInfo.pNext = nullptr;
		queueCreationInfo.flags = 0;
		queueCreationInfo.queueFamilyIndex = index;
		queueCreationInfo.queueCount = 1;
		queueCreationInfo.pQueuePriorities = &fQueuePriority;
		queuesInfo.push_back(queueCreationInfo);
	}

	VkPhysicalDeviceFeatures enabledFeatures{};
	enabledFeatures.samplerAnisotropy = VK_TRUE; // This is the crucial fix

	VkDeviceCreateInfo deviceCreateInfo{};
	deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
	deviceCreateInfo.pNext = nullptr;
	deviceCreateInfo.flags = 0;
	deviceCreateInfo.queueCreateInfoCount = static_cast<uint32_t>(queuesInfo.size());
	deviceCreateInfo.pQueueCreateInfos = queuesInfo.data();

	// Legacy Code
	deviceCreateInfo.enabledLayerCount = 0;
	deviceCreateInfo.ppEnabledLayerNames = nullptr;

	std::vector<const char*> extensions = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
	deviceCreateInfo.enabledExtensionCount = static_cast<uint32_t>(extensions.size());
	deviceCreateInfo.ppEnabledExtensionNames = extensions.data();

	deviceCreateInfo.pEnabledFeatures = &enabledFeatures; // Legacy Code?

	if (vkCreateDevice(m_vkPhysicalDevice, &deviceCreateInfo, nullptr, &m_vkDevice) != VK_SUCCESS)
	{
		syserr("Failed to Create Vulkan Device");
		return (false);
	}

	vkGetDeviceQueue(m_vkDevice, indices.graphicsQueueFamily.value(), 0, &m_vkGraphicsQueue);
	vkGetDeviceQueue(m_vkDevice, indices.presentQueueFamily.value(), 0, &m_vkPresentQueue);

	return (true);
}

bool CVulkanDevice::CreateCommandPool()
{
	SQueueFamilyIndices indices = FindFamilyIndices(GetPhysicalDevice());

	VkCommandPoolCreateInfo cmdPoolCreateInfo{};
	cmdPoolCreateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
	cmdPoolCreateInfo.pNext = VK_NULL_HANDLE;
	cmdPoolCreateInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
	cmdPoolCreateInfo.queueFamilyIndex = indices.graphicsQueueFamily.value();

	if (vkCreateCommandPool(GetDevice(), &cmdPoolCreateInfo, VK_NULL_HANDLE, &m_vkCommandPool) != VK_SUCCESS)
	{
		syserr("Device: Failed to Create Command Pool");
		return (false);
	}

	return (true);
}

uint32_t CVulkanDevice::FindMemoryType(VkPhysicalDevice physicalDevice, uint32_t typeFilter, VkMemoryPropertyFlags properties) const
{
	VkPhysicalDeviceMemoryProperties memProperties;
	vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

	for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++)
	{
		if (typeFilter & (1 << i) && (memProperties.memoryTypes[i].propertyFlags & properties) == properties)
		{
			return (i);
		}
	}

	return (0);
}

void CVulkanDevice::Destroy()
{
	vkDeviceWaitIdle(GetDevice());

	if (m_vkCommandPool != VK_NULL_HANDLE)
	{
		vkDestroyCommandPool(GetDevice(), m_vkCommandPool, VK_NULL_HANDLE);
		m_vkCommandPool = VK_NULL_HANDLE;
	}

	if (m_vkDevice != VK_NULL_HANDLE)
	{
		vkDestroyDevice(m_vkDevice, VK_NULL_HANDLE);
		m_vkDevice = VK_NULL_HANDLE;
	}

	if (m_vkDeviceSurface != VK_NULL_HANDLE)
	{
		vkDestroySurfaceKHR(m_vkInstance, m_vkDeviceSurface, VK_NULL_HANDLE);
		m_vkDeviceSurface = VK_NULL_HANDLE;
	}

	// We can't call the function directly since it's extension
	if (m_vkDebugMessenger != VK_NULL_HANDLE)
	{
		auto funcDestroy = (PFN_vkDestroyDebugUtilsMessengerEXT)vkGetInstanceProcAddr(m_vkInstance, "vkDestroyDebugUtilsMessengerEXT");
		if (funcDestroy)
		{
			funcDestroy(m_vkInstance, m_vkDebugMessenger, VK_NULL_HANDLE);
		}
		m_vkDebugMessenger = VK_NULL_HANDLE;
	}

	if (m_vkInstance != VK_NULL_HANDLE)
	{
		vkDestroyInstance(m_vkInstance, VK_NULL_HANDLE);
		m_vkInstance = VK_NULL_HANDLE;
	}
}

const std::vector<const char*> CVulkanDevice::GetExtensions() const
{
	uint32_t exetnsionsNum = 0;
	const char** glfwExtensions;
	glfwExtensions = glfwGetRequiredInstanceExtensions(&exetnsionsNum);

	std::vector<const char*> allExtensions(glfwExtensions, glfwExtensions + exetnsionsNum);

	// Add Debugging Utils Extension
	allExtensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

	return (allExtensions);
}

bool CVulkanDevice::IsDeviceSuitable(VkPhysicalDevice device)
{
	// Get Device Properties
	VkPhysicalDeviceProperties properties{};
	vkGetPhysicalDeviceProperties(device, &properties);

	VkPhysicalDeviceFeatures features{};
	vkGetPhysicalDeviceFeatures(device, &features);

	if (properties.deviceType != VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU || features.geometryShader == VK_FALSE)
	{
		return (false);
	}

	// Check Queue Family Indices
	SQueueFamilyIndices indices = FindFamilyIndices(device);
	if (indices.IsComplete() == false)
	{
		return (false);
	}

	// Check Device Extensions
	uint32_t extensionsNum = 0;
	if (vkEnumerateDeviceExtensionProperties(device, VK_NULL_HANDLE, &extensionsNum, VK_NULL_HANDLE) != VK_SUCCESS)
	{
		return (false);
	}

	std::vector<VkExtensionProperties> extensionProperties(extensionsNum);
	if (vkEnumerateDeviceExtensionProperties(device, VK_NULL_HANDLE, &extensionsNum, extensionProperties.data()) != VK_SUCCESS)
	{
		return (false);
	}

	// check our needed extension
	for (auto& extension : extensionProperties)
	{
		if (strcmp(extension.extensionName, VK_KHR_SWAPCHAIN_EXTENSION_NAME) == 0)
		{
			return (true);
		}
	}

	return (false);
}

SQueueFamilyIndices CVulkanDevice::FindFamilyIndices(VkPhysicalDevice device)
{
	if (m_gQueueFamilyIndices.IsComplete())
	{
		return (m_gQueueFamilyIndices);
	}

	SQueueFamilyIndices indices{};

	uint32_t queueFamilyProperties = 0;
	vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyProperties, nullptr);

	if (queueFamilyProperties == 0)
	{
		syserr("Failed to Fetch vkGetPhysicalDeviceQueueFamilyProperties");
		return (indices);
	}

	std::vector<VkQueueFamilyProperties> deviceQueueFamilyProperties(queueFamilyProperties);
	vkGetPhysicalDeviceQueueFamilyProperties(device, &queueFamilyProperties, deviceQueueFamilyProperties.data());

	VkBool32 bPresentFamilySupported = false;

	for (uint32_t i = 0; i < deviceQueueFamilyProperties.size(); i++)
	{
		if (deviceQueueFamilyProperties[i].queueFlags & VK_QUEUE_GRAPHICS_BIT)
		{
			indices.graphicsQueueFamily = i;
		}

		vkGetPhysicalDeviceSurfaceSupportKHR(device, i, m_vkDeviceSurface, &bPresentFamilySupported);
		if (bPresentFamilySupported)
		{
			indices.presentQueueFamily = i;
		}

		if (indices.IsComplete())
		{
			break;
		}

	}

	m_gQueueFamilyIndices = indices;
	return (indices);
}

SSwapChainSupportDetails CVulkanDevice::QuerySwapChainSupport()
{
	SSwapChainSupportDetails swapChainDetails{};
	
	vkGetPhysicalDeviceSurfaceCapabilitiesKHR(m_vkPhysicalDevice, m_vkDeviceSurface, &swapChainDetails.capabilities);

	uint32_t formatsCount = 0;
	vkGetPhysicalDeviceSurfaceFormatsKHR(m_vkPhysicalDevice, m_vkDeviceSurface, &formatsCount, VK_NULL_HANDLE);

	if (formatsCount)
	{
		swapChainDetails.formats.resize(formatsCount);
		vkGetPhysicalDeviceSurfaceFormatsKHR(m_vkPhysicalDevice, m_vkDeviceSurface, &formatsCount, swapChainDetails.formats.data());
	}

	uint32_t presentModesCount = 0;
	vkGetPhysicalDeviceSurfacePresentModesKHR(m_vkPhysicalDevice, m_vkDeviceSurface, &presentModesCount, nullptr);

	if (presentModesCount)
	{
		swapChainDetails.presentModes.resize(presentModesCount);
		vkGetPhysicalDeviceSurfacePresentModesKHR(m_vkPhysicalDevice, m_vkDeviceSurface, &presentModesCount, swapChainDetails.presentModes.data());
	}

	return (swapChainDetails);
}

VkFormat CVulkanDevice::FindSupportedFormat(const std::vector<VkFormat>& candidates, VkImageTiling tiling, VkFormatFeatureFlags features)
{
	for (VkFormat format : candidates)
	{
		VkFormatProperties props;
		vkGetPhysicalDeviceFormatProperties(GetPhysicalDevice(), format, &props);

		if (tiling == VK_IMAGE_TILING_LINEAR && (props.linearTilingFeatures & features) == features)
		{
			return format;
		}
		else if (tiling == VK_IMAGE_TILING_OPTIMAL && (props.optimalTilingFeatures & features) == features)
		{
			return format;
		}
	}

	return (VK_FORMAT_UNDEFINED);
}

VkFormat CVulkanDevice::FindDepthFormat()
{
	std::vector<VkFormat> candidates = { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT };
	VkFormat format = FindSupportedFormat(candidates, VK_IMAGE_TILING_OPTIMAL, VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT);
	return (format);
}

bool CVulkanDevice::FormatHasStencilComponent(VkFormat format)
{
	return format == VK_FORMAT_D32_SFLOAT_S8_UINT || format == VK_FORMAT_D24_UNORM_S8_UINT;
}

VKAPI_ATTR VkBool32 VKAPI_CALL CVulkanDevice::DebuggingCallback(VkDebugUtilsMessageSeverityFlagBitsEXT messageSeverity, VkDebugUtilsMessageTypeFlagsEXT messageTypes, const VkDebugUtilsMessengerCallbackDataEXT* pCallbackData, void* pUserData)
{
	if (messageSeverity > VK_DEBUG_UTILS_MESSAGE_SEVERITY_INFO_BIT_EXT)
	{
		syslog("Vulkan: {}", pCallbackData->pMessage);
	}

	if (messageSeverity & VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT)
	{
		// This will trigger a breakpoint in Visual Studio only when an error occurs
		__debugbreak();
	}

	return VK_FALSE;
}

VkInstance CVulkanDevice::GetInstance() const
{
	return (m_vkInstance);
}

VkSurfaceKHR CVulkanDevice::GetDeviceSurface() const
{
	return (m_vkDeviceSurface);
}

VkPhysicalDevice CVulkanDevice::GetPhysicalDevice() const
{
	return (m_vkPhysicalDevice);
}

VkDevice CVulkanDevice::GetDevice() const
{
	return (m_vkDevice);
}

VkQueue CVulkanDevice::GetGraphicsQueue() const
{
	return (m_vkGraphicsQueue);
}

VkQueue CVulkanDevice::GetPresentQueue() const
{
	return (m_vkPresentQueue);
}

uint32_t CVulkanDevice::GetGraphicsQueueFamily()
{
	SQueueFamilyIndices indices = FindFamilyIndices(m_vkPhysicalDevice);
	return (indices.graphicsQueueFamily.value());
}

uint32_t CVulkanDevice::GetPresentQueueFamily()
{
	SQueueFamilyIndices indices = FindFamilyIndices(m_vkPhysicalDevice);
	return (indices.presentQueueFamily.value());
}

VkCommandPool CVulkanDevice::GetCommandPool() const
{
	return (m_vkCommandPool);
}

VkCommandBuffer CVulkanDevice::BeginSingleTimeCommands()
{
	VkCommandBufferAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	allocInfo.commandPool = m_vkCommandPool;
	allocInfo.commandBufferCount = 1;

	VkCommandBuffer commandBuffer;
	vkAllocateCommandBuffers(m_vkDevice, &allocInfo, &commandBuffer);

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

	vkBeginCommandBuffer(commandBuffer, &beginInfo);

	return (commandBuffer);
}

bool CVulkanDevice::EndSingleTimeCommands(VkCommandBuffer commandBuffer)
{
	if (VK_CHECK_BOOL(vkEndCommandBuffer(commandBuffer)) == false)
	{
		return (false);
	}

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &commandBuffer;

	if (VK_CHECK_BOOL(vkQueueSubmit(m_vkGraphicsQueue, 1, &submitInfo, VK_NULL_HANDLE)) == false)
	{
		return (false);
	}
	if (VK_CHECK_BOOL(vkQueueWaitIdle(m_vkGraphicsQueue)) == false)
	{
		return (false);
	}

	// Free Resources
	vkFreeCommandBuffers(m_vkDevice, m_vkCommandPool, 1, &commandBuffer);

	return (true);
}
