#include "Vulkan/VulkanRenderer.h"
#include "Vulkan/VulkanSwapchain.h"
#include "Vulkan/VulkanDescriptorContext.h"
#include "Vulkan/VulkanDescriptorSet.h"
#include "Vulkan/VulkanBuffer.h"
#include "Device/VulkanRenderDevice.h"
#include "Services/RenderQueue.h"
#include "Window/WindowManager.h"
#include "Camera/Camera.h"
#include "Actor/SkeletalActor.h"
#include "Services/ActorsManager.h"
#include "Services/AssimpModelImporter.h"

bool CVulkanRenderer::Initialize()
{
	if (!CreateCommandBuffers())
	{
		return (false);
	}

	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);

	m_pVulkanSyncObject = std::make_unique<CVulkanSyncObject>();
	if (!m_pVulkanSyncObject->Initialize(vkRenderDevice.GetSwapChain()->GetSwapchainImages().size()))
	{
		syserr("Failed to create sync objects");
		return false;
	}

	if (!CreateFrameDescriptorContext())
	{
		syserr("Faile to Initialize Frame Descriptor Context");
		return (false);
	}

	auto& assimpImporter = CServiceLocator::Get<CAssimpModelImporter>();
	auto& actorsMgr = CServiceLocator::Get<CActorsManager>();
	const SActorInfo pInfo = actorsMgr.GetActorInfo("Warrior_Male");
	m_pActor = pInfo.pActor;

	std::shared_ptr<CSkeletalActor> pSkeletalActor = std::dynamic_pointer_cast<CSkeletalActor>(m_pActor);
	pSkeletalActor->GetAnimator()->SetAnimationLibrary(pSkeletalActor->GetSkeletalAsset()->GetAnimations());
	pSkeletalActor->GetAnimator()->PlayAnimation("WarriorMale/OnehandSword/Idle", true);

	return (true);
}

void CVulkanRenderer::Shutdown()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	m_pFrameDescriptorContext->Destroy();

	for (auto& buffer : m_vCameraUBO)
	{
		renderDev.DestroyBuffer(buffer);
		AnubisSafeDelete(buffer);
		buffer = nullptr;
	}

	for (auto& buffer : m_vJointsBuffer)
	{
		renderDev.DestroyBuffer(buffer);
		AnubisSafeDelete(buffer);
		buffer = nullptr;
	}

	m_pVulkanSyncObject->Destroy();

	FreeCommandBuffers();
}

VkCommandBuffer CVulkanRenderer::BeginFrame()
{
	assert(!m_bFrameStarted && "Frame already in progress");

	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	m_pVulkanSyncObject->WaitForFrameFence(m_uiCurrentFrame);

	VkResult result = m_pVulkanSyncObject->AcquireNextImage(context.swapchain, UINT64_MAX, m_uiCurrentFrame, m_uiCurrentImageIndex);

	if (result == VK_ERROR_OUT_OF_DATE_KHR)
	{
		OnResize();
		return VK_NULL_HANDLE;
	}

	if (result != VK_SUCCESS && result != VK_SUBOPTIMAL_KHR)
	{
		syserr("Failed to acquire swapchain image");
		return VK_NULL_HANDLE;
	}

	m_bFrameStarted = true;

	m_pVulkanSyncObject->ResetFrameFence(m_uiCurrentFrame);

	VkCommandBuffer commandBuffer = m_vkvCommandBuffers[m_uiCurrentFrame];
	if (vkResetCommandBuffer(commandBuffer, 0) != VK_SUCCESS)
	{
		syserr("Failed to Reset command buffer");
		return VK_NULL_HANDLE;
	}

	VkCommandBufferBeginInfo beginInfo{};
	beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
	beginInfo.pNext = nullptr;
	beginInfo.flags = 0;
	beginInfo.pInheritanceInfo = nullptr;

	if (vkBeginCommandBuffer(commandBuffer, &beginInfo) != VK_SUCCESS)
	{
		syserr("Failed to begin command buffer");
		m_bFrameStarted = false;
		return VK_NULL_HANDLE;
	}

	return commandBuffer;
}

void CVulkanRenderer::Present()
{
	UpdateRendererBuffers();

	auto& renderQueue = CServiceLocator::Get<CRenderQueue>();
	std::vector<SRenderInstance> renderInstances{};

	if (m_pActor)
	{
		if (m_pActor->GetAsset()->GetModelAsset())
		{
			m_pActor->BuildRenderItemsNew(renderInstances);
			renderQueue.SubimtRenderItems(renderInstances);
		}
	}
}

void CVulkanRenderer::EndFrame()
{
	assert(m_bFrameStarted && "Can't end frame when frame not in progress");

	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);

	VkCommandBuffer commandBuffer = GetCurrentCommandBuffer();

	if (vkEndCommandBuffer(commandBuffer) != VK_SUCCESS)
	{
		syserr("Failed to end command buffer");
		m_bFrameStarted = false;
		return;
	}


	VkPipelineStageFlags waitStages[] = { VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT };
	VkSemaphore waitSemaphores[] = {
		m_pVulkanSyncObject->GetImageAvailableSemaphore(m_uiCurrentFrame)
	};
	VkSemaphore signalSemaphores[] = {
		m_pVulkanSyncObject->GetRenderFinishedSemaphore(m_uiCurrentImageIndex)
	};

	VkSubmitInfo submitInfo{};
	submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
	submitInfo.pNext = nullptr;
	submitInfo.waitSemaphoreCount = 1;
	submitInfo.pWaitSemaphores = waitSemaphores;
	submitInfo.pWaitDstStageMask = waitStages;
	submitInfo.commandBufferCount = 1;
	submitInfo.pCommandBuffers = &commandBuffer;
	submitInfo.signalSemaphoreCount = 1;
	submitInfo.pSignalSemaphores = signalSemaphores;

	if (vkQueueSubmit(
		vkRenderDevice.GetContext().graphicsQueue,
		1,
		&submitInfo,
		m_pVulkanSyncObject->GetFlightFence(m_uiCurrentFrame)) != VK_SUCCESS)
	{
		syserr("Failed to submit draw command buffer");
		m_bFrameStarted = false;
		return;
	}

	VkSwapchainKHR swapChains[] = { vkRenderDevice.GetSwapChain()->GetSwapchain()};

	VkPresentInfoKHR presentInfo{};
	presentInfo.sType = VK_STRUCTURE_TYPE_PRESENT_INFO_KHR;
	presentInfo.pNext = nullptr;
	presentInfo.waitSemaphoreCount = 1;
	presentInfo.pWaitSemaphores = signalSemaphores;
	presentInfo.swapchainCount = 1;
	presentInfo.pSwapchains = swapChains;
	presentInfo.pImageIndices = &m_uiCurrentImageIndex;
	presentInfo.pResults = nullptr;

	VkResult result = vkQueuePresentKHR(vkRenderDevice.GetContext().presentQueue, &presentInfo);

	if (result == VK_ERROR_OUT_OF_DATE_KHR || result == VK_SUBOPTIMAL_KHR || m_bFramebufferResized)
	{
		OnResize();
	}
	else if (result != VK_SUCCESS)
	{
		syserr("Failed to present swapchain image");
	}

	auto& renderQueue = CServiceLocator::Get<CRenderQueue>();
	renderQueue.EndFrame();

	m_bFrameStarted = false;
	m_uiCurrentFrame = (m_uiCurrentFrame + 1) % MAX_FRAMES_IN_FLIGHT;
}

void CVulkanRenderer::FlushRenderItems(ICommandList* pCmd)
{
	if (!pCmd)
	{
		return;
	}

	CVulkanCommandList* vkCmd = dynamic_cast<CVulkanCommandList*>(pCmd);

	if (!vkCmd)
	{
		return;
	}

	auto& renderQueue = CServiceLocator::Get<CRenderQueue>();
	renderQueue.SortRenderItems();
	std::vector<SRenderInstance> renderItems = renderQueue.GetRenderItems();

	for (const auto& renderItem : renderItems)
	{
		const auto batch = renderItem.pBatch;

		vkCmd->BindPipeline(batch->pPipeline);
		vkCmd->BindVertexBuffer(batch->pVertexBuffer);
		vkCmd->BindIndexBuffer(batch->pIndexBuffer, EIndexType::INDEX_TYPE_UINT32);
		vkCmd->BindMaterial(batch->pMaterial, GetCurrentFrameIndex()); // set 0
		vkCmd->BindFrameDescriptorSet(m_pFrameDescriptorContext->GetDescriptorSet(GetCurrentFrameIndex()), EBiningLayoutSetsPoints::BINDING_POINT_FRAME_RESOURCES); // set 1

		SUniformBufferBlockModel modelData{};
		modelData.matModel = renderItem.modelMatrix;
		modelData.skinPaletteIndex = renderItem.skinPaletteIndex;
		modelData.flags = renderItem.flags;
		vkCmd->PushConstants(&modelData, sizeof(modelData));
		vkCmd->DrawIndexed(batch->indexCount, 1, batch->firstIndex);
	}

	m_vRenderItems.clear();
}

void CVulkanRenderer::BeginSwapchainRenderPass(VkCommandBuffer commandBuffer)
{
	assert(m_bFrameStarted && "Can't begin render pass when frame not in progress");
	assert(commandBuffer == GetCurrentCommandBuffer() && "Command buffer is not current frame buffer");

	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	VkRenderPassBeginInfo renderPassBeginInfo{};
	renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
	renderPassBeginInfo.pNext = nullptr;
	renderPassBeginInfo.renderPass = vkRenderDevice.GetSwapChain()->GetRenderPass();
	renderPassBeginInfo.framebuffer = vkRenderDevice.GetSwapChain()->GetFramebuffer(m_uiCurrentImageIndex);
	renderPassBeginInfo.renderArea.offset = { 0, 0 };
	renderPassBeginInfo.renderArea.extent = vkRenderDevice.GetSwapChain()->GetSwapchainExtent();

	std::array<VkClearValue, 2> clearValues{};
	clearValues[0].color = { {0.1f, 0.1f, 0.1f, 1.0f} };
	clearValues[1].depthStencil = { 1.0f, 0 };

	renderPassBeginInfo.clearValueCount = static_cast<uint32_t>(clearValues.size());
	renderPassBeginInfo.pClearValues = clearValues.data();

	vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo, VK_SUBPASS_CONTENTS_INLINE);

	VkViewport viewport{};
	viewport.x = 0.0f;
	viewport.y = 0.0f;
	viewport.width = static_cast<float>(vkRenderDevice.GetSwapChain()->GetSwapchainExtent().width);
	viewport.height = static_cast<float>(vkRenderDevice.GetSwapChain()->GetSwapchainExtent().height);
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;

	VkRect2D scissor{};
	scissor.offset = { 0, 0 };
	scissor.extent = vkRenderDevice.GetSwapChain()->GetSwapchainExtent();

	vkCmdSetViewport(commandBuffer, 0, 1, &viewport);
	vkCmdSetScissor(commandBuffer, 0, 1, &scissor);
}

void CVulkanRenderer::EndSwapchainRenderPass(VkCommandBuffer commandBuffer)
{
	assert(m_bFrameStarted && "Can't end render pass when frame not in progress");
	assert(commandBuffer == GetCurrentCommandBuffer() && "Command buffer is not current frame buffer");

	vkCmdEndRenderPass(commandBuffer);
}

bool CVulkanRenderer::IsFrameInProgress() const
{
	return m_bFrameStarted;
}

VkCommandBuffer CVulkanRenderer::GetCurrentCommandBuffer() const
{
	return m_bFrameStarted ? m_vkvCommandBuffers[m_uiCurrentFrame] : VK_NULL_HANDLE;
}

uint32_t CVulkanRenderer::GetCurrentFrameIndex() const
{
	return m_uiCurrentFrame;
}

uint32_t CVulkanRenderer::GetCurrentImageIndex() const
{
	return m_uiCurrentImageIndex;
}

void CVulkanRenderer::OnResize()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	vkDeviceWaitIdle(context.device);

	m_bFramebufferResized = false;

	RecreateSwapchain();
}

std::vector<SRenderInstance>& CVulkanRenderer::GetRenderItems()
{
	return (m_vRenderItems);
}

const std::vector<SRenderInstance>& CVulkanRenderer::GetRenderItems() const
{
	return (m_vRenderItems);
}

bool CVulkanRenderer::CreateCommandBuffers()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	m_vkvCommandBuffers.resize(MAX_FRAMES_IN_FLIGHT);

	VkCommandBufferAllocateInfo cmdBufferAllocateInfo{};
	cmdBufferAllocateInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
	cmdBufferAllocateInfo.pNext = nullptr;
	cmdBufferAllocateInfo.commandPool = context.commandPool;
	cmdBufferAllocateInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
	cmdBufferAllocateInfo.commandBufferCount = static_cast<uint32_t>(m_vkvCommandBuffers.size());

	if (VK_CHECK_BOOL(vkAllocateCommandBuffers(context.device, &cmdBufferAllocateInfo, m_vkvCommandBuffers.data())) == false)
	{
		syserr("Renderer: Failed to Allocate Command Buffer");
		return (false);
	}

	return (true);
}

void CVulkanRenderer::FreeCommandBuffers()
{
	if (m_vkvCommandBuffers.empty())
	{
		return;
	}

	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	if (context.device == VK_NULL_HANDLE || context.commandPool == VK_NULL_HANDLE)
	{
		return; // ??? what are you doing
	}

	vkFreeCommandBuffers(context.device, context.commandPool, static_cast<uint32_t>(m_vkvCommandBuffers.size()), m_vkvCommandBuffers.data());
	m_vkvCommandBuffers.clear();
}

bool CVulkanRenderer::RecreateSwapchain()
{
	FreeCommandBuffers();

	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);

	if (!vkRenderDevice.RecreateSwapchain())
	{
		return (false);
	}

	if (!CreateCommandBuffers())
	{
		return (false);
	}

	return true;
}

bool CVulkanRenderer::CreateFrameDescriptorContext()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

	// Initialize Buffers
	m_vCameraUBO.reserve(MAX_FRAMES_IN_FLIGHT);
	m_vJointsBuffer.reserve(MAX_FRAMES_IN_FLIGHT);

	for (size_t i = 0; i < MAX_FRAMES_IN_FLIGHT; i++)
	{
		SBufferDesc cameraUBODesc{};
		cameraUBODesc.m_stName = "Camera Uniform Buffer";
		cameraUBODesc.m_eType = EBufferType::BUFFER_TYPE_UNIFORM;
		cameraUBODesc.m_uiSize = sizeof(SUniformBufferBlock);
		cameraUBODesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
		cameraUBODesc.cpuWrite = true;

		IBuffer* pCameraBuffer = renderDev.CreateBuffer(cameraUBODesc);
		if (!pCameraBuffer)
		{
			syserr("Failed to Create Camera Buffer");
			return (false);
		}

		m_vCameraUBO.push_back(pCameraBuffer);

		uint32_t initialSize = 1 * sizeof(Matrix4);

		SBufferDesc joinsBufferDesc{};
		joinsBufferDesc.m_stName = "Model Storage Uniform Buffer";
		joinsBufferDesc.m_eType = EBufferType::BUFFER_TYPE_STORAGE;
		joinsBufferDesc.m_uiSize = initialSize; // Initialize as 100 Metrices
		joinsBufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
		joinsBufferDesc.cpuWrite = true;

		IBuffer* pJoinsBuffer = renderDev.CreateBuffer(joinsBufferDesc);
		if (!pJoinsBuffer)
		{
			syserr("Failed to Create Joints Buffer");
			return (false);
		}

		m_vJointsBuffer.push_back(pJoinsBuffer);
	}


	SBindingContextDesc ctxDesc{};
	ctxDesc.m_uiFrameCount = MAX_FRAMES_IN_FLIGHT;
	ctxDesc.m_vBindings = CDescriptorSetLayouts::GetFrameBindings();

	ctxDesc.m_vBufferResources.push_back({
		static_cast<uint32_t>(EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_CAMERA_UBO),
		m_vCameraUBO
		});

	ctxDesc.m_vBufferResources.push_back({
		static_cast<uint32_t>(EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_BONES_SSBO),
		m_vJointsBuffer
		});

	m_pFrameDescriptorContext = std::make_unique<CVulkanDescriptorContext>();
	if (!m_pFrameDescriptorContext->Initialize(ctxDesc))
	{
		return (false);
	}

	return (true);
}

void CVulkanRenderer::UpdateRendererBuffers()
{
	auto& windowMgr = CServiceLocator::Get<CWindowManager>();
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);

	static auto startTime = std::chrono::high_resolution_clock::now();
	auto currentTime = std::chrono::high_resolution_clock::now();
	float time = std::chrono::duration<float, std::chrono::seconds::period>(currentTime - startTime).count();

	SUniformBufferBlock bufferBlock{};

	bufferBlock.matView = windowMgr.GetCamera()->GetViewMatrix();

	bufferBlock.matProjection = windowMgr.GetCamera()->GetProjectionMatrix();
	bufferBlock.matProjection[1][1] *= -1.0f;

	bufferBlock.matViewProjection = bufferBlock.matProjection * bufferBlock.matView;

	renderDev.UpdateBuffer(m_vCameraUBO[GetCurrentFrameIndex()], &bufferBlock, sizeof(SUniformBufferBlock), 0);

	// Update Joints Buffer
	std::shared_ptr<CSkeletalActor> pSkeletalActor = std::dynamic_pointer_cast<CSkeletalActor>(m_pActor);
	std::vector<Matrix4> jointsMertices = pSkeletalActor->GetAnimator()->GetFinalBoneMatrices();

	uint64_t jointsBufferSize = jointsMertices.size() * sizeof(Matrix4);
	SBufferDesc joinsBufferDesc{};
	joinsBufferDesc.m_stName = "Model Joints Storage Buffer";
	joinsBufferDesc.m_eType = EBufferType::BUFFER_TYPE_STORAGE;
	joinsBufferDesc.m_uiSize = jointsBufferSize; // Initialize as 100 Metrices
	joinsBufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
	joinsBufferDesc.cpuWrite = true;

	EBufferResizeResult result = vkRenderDevice.EnsureBufferCapacity(m_vJointsBuffer[GetCurrentFrameIndex()], jointsBufferSize, joinsBufferDesc);
	if (result == EBufferResizeResult::BUFFER_RESIZE_FAILED)
	{
		return;
	}

	if (result == EBufferResizeResult::BUFFER_RESIZE_RECREATED)
	{
		uint32_t bindingPoint = static_cast<uint32_t>(EBufferBindingPointsSetOne::BINDING_POINT_SET_ONE_BONES_SSBO);
		m_pFrameDescriptorContext->UpdateBufferBinding(GetCurrentFrameIndex(), bindingPoint, m_vJointsBuffer[GetCurrentFrameIndex()], 0, jointsBufferSize); // or VK_WHOLE_SIZE
	}

	renderDev.UpdateBuffer(m_vJointsBuffer[GetCurrentFrameIndex()], jointsMertices.data(), jointsBufferSize, 0);
}
