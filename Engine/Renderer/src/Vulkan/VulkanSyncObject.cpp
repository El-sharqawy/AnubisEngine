#include "Vulkan/VulkanSyncObject.h"
#include "Vulkan/VulkanDevice.h"
#include "Vulkan/VulkanSwapchain.h"
#include "Device/VulkanRenderDevice.h"

bool CVulkanSyncObject::Initialize(size_t swapChainImages)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	m_vkvImagesAvailableSemaphores.resize(MAX_FRAMES_IN_FLIGHT);
	m_vkvFlightFences.resize(MAX_FRAMES_IN_FLIGHT);
	
	m_uiSwapchainImageCount = swapChainImages;
	m_vkvRenderingFinishedSemaphores.resize(m_uiSwapchainImageCount);

	VkSemaphoreCreateInfo semaphoreCreateInfo{};
	semaphoreCreateInfo.sType = VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO;
	semaphoreCreateInfo.pNext = VK_NULL_HANDLE;
	semaphoreCreateInfo.flags = 0;

	VkFenceCreateInfo fenceCreateInfo{};
	fenceCreateInfo.sType = VK_STRUCTURE_TYPE_FENCE_CREATE_INFO;
	fenceCreateInfo.pNext = VK_NULL_HANDLE;
	fenceCreateInfo.flags = VK_FENCE_CREATE_SIGNALED_BIT;

	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
	{
		if (VK_CHECK_BOOL(vkCreateSemaphore(context.device, &semaphoreCreateInfo, VK_NULL_HANDLE, &m_vkvImagesAvailableSemaphores[i])) == false)
		{
			syserr("Failed to Create Image Available Semaphore");
			return (false);
		}
		if (VK_CHECK_BOOL(vkCreateFence(context.device, &fenceCreateInfo, VK_NULL_HANDLE, &m_vkvFlightFences[i])) == false)
		{
			syserr("Failed to Create Rendering Flight Fence");
			return (false);
		}
	}

	for (size_t i = 0; i < m_uiSwapchainImageCount; i++)
	{
		if (VK_CHECK_BOOL(vkCreateSemaphore(context.device, &semaphoreCreateInfo, VK_NULL_HANDLE, &m_vkvRenderingFinishedSemaphores[i])) == false)
		{
			syserr("Failed to Create Rendering Finished Semaphore");
			return (false);
		}
	}

	return (true);
}

void CVulkanSyncObject::Destroy()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
	{
		if (m_vkvImagesAvailableSemaphores[i] != VK_NULL_HANDLE)
		{
			vkDestroySemaphore(context.device, m_vkvImagesAvailableSemaphores[i], VK_NULL_HANDLE);
			m_vkvImagesAvailableSemaphores[i] = VK_NULL_HANDLE;
		}
		if (m_vkvFlightFences[i] != VK_NULL_HANDLE)
		{
			vkDestroyFence(context.device, m_vkvFlightFences[i], VK_NULL_HANDLE);
			m_vkvFlightFences[i] = VK_NULL_HANDLE;
		}
	}

	uint32_t swapchainImageCount = m_uiSwapchainImageCount;
	for (size_t i = 0; i < swapchainImageCount; i++)
	{
		if (m_vkvRenderingFinishedSemaphores[i] != VK_NULL_HANDLE)
		{
			vkDestroySemaphore(context.device, m_vkvRenderingFinishedSemaphores[i], VK_NULL_HANDLE);
			m_vkvRenderingFinishedSemaphores[i] = VK_NULL_HANDLE;
		}
	}
}

void CVulkanSyncObject::WaitForFrameFence(uint32_t frameIndex)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	if (vkWaitForFences(context.device, 1, &m_vkvFlightFences[frameIndex], VK_TRUE, UINT64_MAX) != VK_SUCCESS)
	{
		return;
	}
}

void CVulkanSyncObject::ResetFrameFence(uint32_t frameIndex)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	if (vkResetFences(context.device, 1, &m_vkvFlightFences[frameIndex]) != VK_SUCCESS)
	{
	}
}

VkResult CVulkanSyncObject::AcquireNextImage(VkSwapchainKHR swapChain, uint64_t timeout, uint32_t frameIndex, uint32_t& imageIndex)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	VkResult resultAcquireNextImg = vkAcquireNextImageKHR(context.device, swapChain, timeout, m_vkvImagesAvailableSemaphores[frameIndex], VK_NULL_HANDLE, &imageIndex);
	if (resultAcquireNextImg == VK_ERROR_OUT_OF_DATE_KHR || resultAcquireNextImg == VK_SUBOPTIMAL_KHR)
	{
		syserr("Swapchain is out of date or suboptimal");
	}
	else if (resultAcquireNextImg != VK_SUCCESS)
	{
	}
	return (resultAcquireNextImg);
}

VkSemaphore CVulkanSyncObject::GetImageAvailableSemaphore(uint32_t frameIndex)
{
	return (m_vkvImagesAvailableSemaphores.at(frameIndex));
}

VkSemaphore CVulkanSyncObject::GetRenderFinishedSemaphore(uint32_t frameIndex)
{
	return (m_vkvRenderingFinishedSemaphores.at(frameIndex));
}

VkFence CVulkanSyncObject::GetFlightFence(uint32_t frameIndex)
{
	return (m_vkvFlightFences.at(frameIndex));
}
