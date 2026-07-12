#include "Vulkan/VulkanSwapchain.h"
#include "Vulkan/VulkanDevice.h"
#include "Window/WindowManager.h"
#include "Device/VulkanRenderDevice.h"
#include <algorithm>

bool CVulkanSwapchain::InitializeSwapchain()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	SSwapChainSupportDetails swapChainDetails = vkRenderDevice.QuerySwapChainSupport();
	VkSurfaceFormatKHR surfaceFormat = ChooseSurfaceFormat(swapChainDetails.formats);
	VkPresentModeKHR presentMode = ChoosePresentMode(swapChainDetails.presentModes);
	VkExtent2D extent = ChooseExtent(swapChainDetails.capabilities);

	VkSwapchainCreateInfoKHR swapChainCreateInfo{};
	swapChainCreateInfo.sType = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
	swapChainCreateInfo.pNext = VK_NULL_HANDLE;
	swapChainCreateInfo.flags = 0; // no flags needed
	swapChainCreateInfo.surface = context.surface;
	
	uint32_t imageCount = swapChainDetails.capabilities.minImageCount + 1;
	if (swapChainDetails.capabilities.maxImageCount > 0 && swapChainDetails.capabilities.maxImageCount > imageCount)
	{
		imageCount = swapChainDetails.capabilities.maxImageCount;
	}

	swapChainCreateInfo.minImageCount = imageCount;
	swapChainCreateInfo.imageFormat = surfaceFormat.format;
	swapChainCreateInfo.imageColorSpace = surfaceFormat.colorSpace;
	swapChainCreateInfo.imageExtent = extent;
	swapChainCreateInfo.imageArrayLayers = 1;
	swapChainCreateInfo.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

	SQueueFamilyIndices indices = vkRenderDevice.FindFamilyIndices(vkRenderDevice.GetPhysicalDevice());
	uint32_t queueFamily[] = {indices.graphicsQueueFamily.value(), indices.presentQueueFamily.value()};

	if (indices.graphicsQueueFamily != indices.presentQueueFamily)
	{
		swapChainCreateInfo.imageSharingMode = VK_SHARING_MODE_CONCURRENT;
		swapChainCreateInfo.queueFamilyIndexCount = 2;
		swapChainCreateInfo.pQueueFamilyIndices = queueFamily;
	}
	else
	{
		swapChainCreateInfo.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
		swapChainCreateInfo.queueFamilyIndexCount = 0;
		swapChainCreateInfo.pQueueFamilyIndices = VK_NULL_HANDLE;
	}

	swapChainCreateInfo.preTransform = swapChainDetails.capabilities.currentTransform;
	swapChainCreateInfo.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR; // we dont care about alpha
	swapChainCreateInfo.presentMode = presentMode;
	swapChainCreateInfo.clipped = VK_TRUE; // clip when window is minimzed for example
	swapChainCreateInfo.oldSwapchain = VK_NULL_HANDLE; // neeed to assign it on window resize

	if (vkCreateSwapchainKHR(context.device, &swapChainCreateInfo, VK_NULL_HANDLE, &m_vkSwapchain) != VK_SUCCESS)
	{
		syserr("Failed to Create swapchain");
		return (false);
	}

	m_vkSwapchainImageFormat = surfaceFormat.format;
	m_vkSwapchainExtent = extent;

	// Get Swapchain Images
	uint32_t swapChainCount;
	vkGetSwapchainImagesKHR(context.device, m_vkSwapchain, &swapChainCount, VK_NULL_HANDLE);
	m_vkvSwapchainImages.resize(swapChainCount);
	vkGetSwapchainImagesKHR(context.device, m_vkSwapchain, &swapChainCount, m_vkvSwapchainImages.data());

	return (true);
}

bool CVulkanSwapchain::CreateImageViews()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	m_vkvSwapChainImageViews.resize(m_vkvSwapchainImages.size());
	for (int32_t i = 0; i < m_vkvSwapchainImages.size(); i++)
	{
		VkImageViewCreateInfo imageViewInfo{};
		imageViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
		imageViewInfo.pNext = VK_NULL_HANDLE;
		imageViewInfo.flags = 0;
		imageViewInfo.image = m_vkvSwapchainImages[i];
		imageViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
		imageViewInfo.format = m_vkSwapchainImageFormat;
		imageViewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
		imageViewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
		imageViewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
		imageViewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
		imageViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
		imageViewInfo.subresourceRange.baseMipLevel = 0;
		imageViewInfo.subresourceRange.levelCount = 1;
		imageViewInfo.subresourceRange.baseArrayLayer = 0;
		imageViewInfo.subresourceRange.layerCount = 1;

		if (VK_CHECK_BOOL(vkCreateImageView(context.device, &imageViewInfo, VK_NULL_HANDLE, &m_vkvSwapChainImageViews[i])) == false)
		{
			syserr("Failed to Create Texture Image View");
			return (false);
		}
	}

	return (true);
}

bool CVulkanSwapchain::CreateDepthResources()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	VkFormat depthFormat = vkRenderDevice.FindDepthFormat();

	VkImageCreateInfo imageInfo{};
	imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
	imageInfo.imageType = VK_IMAGE_TYPE_2D;
	imageInfo.extent.width = static_cast<uint32_t>(m_vkSwapchainExtent.width);
	imageInfo.extent.height = static_cast<uint32_t>(m_vkSwapchainExtent.height);
	imageInfo.extent.depth = 1;
	imageInfo.mipLevels = 1;
	imageInfo.arrayLayers = 1;
	imageInfo.format = depthFormat;
	imageInfo.tiling = VK_IMAGE_TILING_OPTIMAL;
	imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	imageInfo.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
	imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
	imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
	imageInfo.flags = 0; // Optional

	if (VK_CHECK_BOOL(vkCreateImage(context.device, &imageInfo, nullptr, &m_vkDepthImage)) == false)
	{
		syserr("failed to create Vulkan image for depth image!");
		return (false);
	}

	// Allocate Memory for the Image
	VkMemoryRequirements memRequirements;
	vkGetImageMemoryRequirements(context.device, m_vkDepthImage, &memRequirements);

	VkMemoryAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
	allocInfo.allocationSize = memRequirements.size;
	allocInfo.memoryTypeIndex = vkRenderDevice.FindMemoryType(vkRenderDevice.GetPhysicalDevice(), memRequirements.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

	if (VK_CHECK_BOOL(vkAllocateMemory(context.device, &allocInfo, nullptr, &m_vkDepthImageMemory)) == false)
	{
		syserr("failed to allocate depth image memory!");
		return (false);
	}

	// Bind the Image Memory
	if (VK_CHECK_BOOL(vkBindImageMemory(context.device, m_vkDepthImage, m_vkDepthImageMemory, 0)) == false)
	{
		syserr("failed to bind depth image memory");
		return (false);
	}

	// Create Depth Image View
	VkImageViewCreateInfo imageViewInfo{};
	imageViewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
	imageViewInfo.pNext = VK_NULL_HANDLE;
	imageViewInfo.flags = 0;
	imageViewInfo.image = m_vkDepthImage;
	imageViewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
	imageViewInfo.format = depthFormat;
	imageViewInfo.components.r = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewInfo.components.g = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewInfo.components.b = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewInfo.components.a = VK_COMPONENT_SWIZZLE_IDENTITY;
	imageViewInfo.subresourceRange.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
	imageViewInfo.subresourceRange.baseMipLevel = 0;
	imageViewInfo.subresourceRange.levelCount = 1;
	imageViewInfo.subresourceRange.baseArrayLayer = 0;
	imageViewInfo.subresourceRange.layerCount = 1;

	if (VK_CHECK_BOOL(vkCreateImageView(context.device, &imageViewInfo, VK_NULL_HANDLE, &m_vkDepthImageView)) == false)
	{
		syserr("Failed to Create Depth Image View");
		return (false);
	}

	vkRenderDevice.TransitionImageLayout(m_vkDepthImage, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL);

	return (true);
}

bool CVulkanSwapchain::CreateRenderPass()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	// Create Color RenderPass
	VkAttachmentDescription colorAttachment{};
	colorAttachment.flags = 0;
	colorAttachment.format = GetSwapchainImageFormat();
	colorAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	colorAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	colorAttachment.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
	colorAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	colorAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	colorAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	colorAttachment.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

	// Create Depth RenderPass
	VkAttachmentDescription depthAttachment{};
	depthAttachment.flags = 0;
	depthAttachment.format = vkRenderDevice.FindDepthFormat();
	depthAttachment.samples = VK_SAMPLE_COUNT_1_BIT;
	depthAttachment.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
	depthAttachment.storeOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
	depthAttachment.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
	depthAttachment.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
	depthAttachment.finalLayout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	// Color Attachment Ref
	VkAttachmentReference colorAttachmentRef{};
	colorAttachmentRef.attachment = 0;
	colorAttachmentRef.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;

	// Depth Attachment Ref
	VkAttachmentReference depthAttachmentRef{};
	depthAttachmentRef.attachment = 1;
	depthAttachmentRef.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

	// SubpassDescription
	VkSubpassDescription subpassDesc{};
	subpassDesc.flags = 0;
	subpassDesc.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
	subpassDesc.inputAttachmentCount = 0;
	subpassDesc.pInputAttachments = VK_NULL_HANDLE;
	subpassDesc.colorAttachmentCount = 1;
	subpassDesc.pColorAttachments = &colorAttachmentRef;
	subpassDesc.pResolveAttachments = VK_NULL_HANDLE;
	subpassDesc.pDepthStencilAttachment = &depthAttachmentRef;
	subpassDesc.preserveAttachmentCount = 0;
	subpassDesc.pPreserveAttachments = VK_NULL_HANDLE;

	VkSubpassDependency subpassDependency{};
	subpassDependency.srcSubpass = VK_SUBPASS_EXTERNAL;
	subpassDependency.dstSubpass = 0;

	subpassDependency.srcStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_LATE_FRAGMENT_TESTS_BIT;
	subpassDependency.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
	subpassDependency.srcAccessMask = VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
	subpassDependency.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;
	subpassDependency.dependencyFlags = 0;

	std::array<VkAttachmentDescription, 2> attachments = { colorAttachment, depthAttachment };

	VkRenderPassCreateInfo renderPassCreateInfo{};
	renderPassCreateInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
	renderPassCreateInfo.pNext = VK_NULL_HANDLE;
	renderPassCreateInfo.flags = 0;
	renderPassCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
	renderPassCreateInfo.pAttachments = attachments.data();
	renderPassCreateInfo.subpassCount = 1;
	renderPassCreateInfo.pSubpasses = &subpassDesc;
	renderPassCreateInfo.dependencyCount = 1;
	renderPassCreateInfo.pDependencies = &subpassDependency;

	if (vkCreateRenderPass(context.device, &renderPassCreateInfo, VK_NULL_HANDLE, &m_vkRenderPass) != VK_SUCCESS)
	{
		syserr("Failed to Create RenderPass");
		return (false);
	}

	return (true);
}

bool CVulkanSwapchain::CreateFramebuffers()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	m_vkvSwapchainFrameBuffers.resize(GetSwapchainImageViews().size());

	std::vector<VkImageView> vImageViews = GetSwapchainImageViews();

	for (size_t i = 0; i < m_vkvSwapchainFrameBuffers.size(); i++)
	{
		std::array<VkImageView, 2> attachments = {
			vImageViews[i],
			GetDepthImageView()
		};

		VkFramebufferCreateInfo framebufferCreateInfo{};
		framebufferCreateInfo.sType = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
		framebufferCreateInfo.pNext = VK_NULL_HANDLE;
		framebufferCreateInfo.flags = 0;
		framebufferCreateInfo.renderPass = GetRenderPass();
		framebufferCreateInfo.attachmentCount = static_cast<uint32_t>(attachments.size());
		framebufferCreateInfo.pAttachments = attachments.data();
		framebufferCreateInfo.width = GetSwapchainExtent().width;
		framebufferCreateInfo.height = GetSwapchainExtent().height;
		framebufferCreateInfo.layers = 1;

		if (VK_CHECK_BOOL(vkCreateFramebuffer(context.device, &framebufferCreateInfo, VK_NULL_HANDLE, &m_vkvSwapchainFrameBuffers[i])) == false)
		{
			syserr("Failed to Create framebuffer at index %zu", i);
			return (false);
		}
	}

	return (true);
}

void CVulkanSwapchain::DestroyFramebuffers()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	for (auto& framebuffer : m_vkvSwapchainFrameBuffers)
	{
		if (framebuffer != VK_NULL_HANDLE)
		{
			vkDestroyFramebuffer(context.device, framebuffer, nullptr);
			framebuffer = VK_NULL_HANDLE;
		}
	}

	m_vkvSwapchainFrameBuffers.clear();
}

void CVulkanSwapchain::DestroyDepthResources()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	if (m_vkDepthImage != VK_NULL_HANDLE)
	{
		vkDestroyImage(context.device, m_vkDepthImage, VK_NULL_HANDLE);
		m_vkDepthImage = VK_NULL_HANDLE;
	}

	if (m_vkDepthImageMemory != VK_NULL_HANDLE)
	{
		vkFreeMemory(context.device, m_vkDepthImageMemory, VK_NULL_HANDLE);
		m_vkDepthImageMemory = VK_NULL_HANDLE;
	}

	if (m_vkDepthImageView != VK_NULL_HANDLE)
	{
		vkDestroyImageView(context.device, m_vkDepthImageView, VK_NULL_HANDLE);
		m_vkDepthImageView = VK_NULL_HANDLE;

	}
}

void CVulkanSwapchain::Clear()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	DestroyFramebuffers();
	DestroyDepthResources();

	if (m_vkRenderPass != VK_NULL_HANDLE)
	{
		vkDestroyRenderPass(context.device, m_vkRenderPass, VK_NULL_HANDLE);
		m_vkRenderPass = VK_NULL_HANDLE;
	}

	for (auto& imageView : m_vkvSwapChainImageViews)
	{
		vkDestroyImageView(context.device, imageView, VK_NULL_HANDLE);
		imageView = VK_NULL_HANDLE;
	}

	if (m_vkSwapchain != VK_NULL_HANDLE)
	{
		vkDestroySwapchainKHR(context.device, m_vkSwapchain, VK_NULL_HANDLE);
		m_vkSwapchain = VK_NULL_HANDLE;
	}
}

void CVulkanSwapchain::Destroy()
{
	Clear();
}

VkSwapchainKHR CVulkanSwapchain::GetSwapchain() const
{
	return (m_vkSwapchain);
}

VkFormat CVulkanSwapchain::GetSwapchainImageFormat() const
{
	return (m_vkSwapchainImageFormat);
}

VkExtent2D CVulkanSwapchain::GetSwapchainExtent() const
{
	return (m_vkSwapchainExtent);
}

const std::vector<VkImage>& CVulkanSwapchain::GetSwapchainImages() const
{
	return (m_vkvSwapchainImages);
}

const std::vector<VkImageView>& CVulkanSwapchain::GetSwapchainImageViews() const
{
	return (m_vkvSwapChainImageViews);
}

const std::vector<VkFramebuffer>& CVulkanSwapchain::GetFramebuffers() const
{
	return (m_vkvSwapchainFrameBuffers);
}

VkImage CVulkanSwapchain::GetDepthImage() const
{
	return (m_vkDepthImage);
}

VkDeviceMemory CVulkanSwapchain::GetDepthImageMemory() const
{
	return (m_vkDepthImageMemory);
}

VkImageView CVulkanSwapchain::GetDepthImageView() const
{
	return (m_vkDepthImageView);
}

VkRenderPass CVulkanSwapchain::GetRenderPass() const
{
	return (m_vkRenderPass);
}

VkFramebuffer CVulkanSwapchain::GetFramebuffer(uint32_t imageIndex) const
{
	if (imageIndex >= m_vkvSwapchainFrameBuffers.size())
	{
		return VK_NULL_HANDLE;
	}

	return (m_vkvSwapchainFrameBuffers.at(imageIndex));
}

VkSurfaceFormatKHR CVulkanSwapchain::ChooseSurfaceFormat(const std::vector<VkSurfaceFormatKHR>& vFormats)
{
	for (auto& format : vFormats)
	{
		if (format.format == VK_FORMAT_R8G8B8A8_SRGB && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
		{
			return format;
		}
		else if (format.format == VK_FORMAT_R8G8B8A8_SRGB && format.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
		{
			return (format);
		}
	}
	return vFormats.at(0);
}

VkPresentModeKHR CVulkanSwapchain::ChoosePresentMode(const std::vector<VkPresentModeKHR>& presentModes)
{
	for (auto& presentMode : presentModes)
	{
		if (presentMode == VK_PRESENT_MODE_MAILBOX_KHR)
		{
			return presentMode;
		}
		else if (presentMode == VK_PRESENT_MODE_FIFO_KHR)
		{
			return presentMode;
		}
	}
	return presentModes.at(0);
}

VkExtent2D CVulkanSwapchain::ChooseExtent(const VkSurfaceCapabilitiesKHR& capabilities)
{
	if (capabilities.currentExtent.width != UINT32_MAX)
	{
		return capabilities.currentExtent;
	}
	else
	{
		int32_t width, height;
		auto& windowMgr = CWindowManager::Instance();
		glfwGetFramebufferSize(windowMgr.GetGLFWWindow(), &width, &height);

		VkExtent2D actualExtent = { static_cast<uint32_t>(width), static_cast<uint32_t>(height) };
		actualExtent.width = std::clamp(static_cast<uint32_t>(actualExtent.width), capabilities.minImageExtent.width, capabilities.maxImageExtent.width);
		actualExtent.height = std::clamp(static_cast<uint32_t>(actualExtent.height), capabilities.minImageExtent.height, capabilities.maxImageExtent.height);

		return (actualExtent);
	}
}