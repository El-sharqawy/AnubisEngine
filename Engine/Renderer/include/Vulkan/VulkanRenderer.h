#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include "API/RenderObject.h"
#include "API/ActorData.h"
#include "Vulkan/VulkanSyncObject.h"
#include "Vulkan/VulkanDescriptorContext.h"

class CVulkanDescriptorContext;
class ICommandList;

class CVulkanRenderer
{
public:
    CVulkanRenderer() = default;
    ~CVulkanRenderer() = default;

    bool Initialize();
    void Shutdown();

    VkCommandBuffer BeginFrame();
    void Present();
    void EndFrame();

    // Present
    void FlushRenderItems(ICommandList* pCmd);

    void BeginSwapchainRenderPass(VkCommandBuffer commandBuffer);
    void EndSwapchainRenderPass(VkCommandBuffer commandBuffer);

    bool IsFrameInProgress() const;
    VkCommandBuffer GetCurrentCommandBuffer() const;
    uint32_t GetCurrentFrameIndex() const;
    uint32_t GetCurrentImageIndex() const;

    void OnResize();

    std::vector<SRenderInstance>& GetRenderItems();
    const std::vector<SRenderInstance>& GetRenderItems() const;

private:
    bool CreateCommandBuffers();
    void FreeCommandBuffers();
    bool RecreateSwapchain();
    bool CreateFrameDescriptorContext();
    void UpdateRendererBuffers();

private:
    bool m_bFrameStarted = false;
    uint32_t m_uiCurrentFrame = 0;
    uint32_t m_uiCurrentImageIndex = 0;
    std::vector<VkCommandBuffer> m_vkvCommandBuffers;
    std::unique_ptr<CVulkanSyncObject> m_pVulkanSyncObject = nullptr;
    bool m_bFramebufferResized = false;
    std::vector<SRenderInstance> m_vRenderItems = {};

    // Camera Matrix
    std::unique_ptr<CVulkanDescriptorContext> m_pFrameDescriptorContext;
    std::vector<IBuffer*> m_vCameraUBO = {};
    std::vector<IBuffer*> m_vJointsBuffer = {};
};
