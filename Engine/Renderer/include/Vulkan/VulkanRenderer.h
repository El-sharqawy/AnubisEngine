#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include "API/RenderObject.h"
#include "Vulkan/VulkanSyncObject.h"

class ICommandList;

class CVulkanRenderer
{
public:
    CVulkanRenderer() = default;
    ~CVulkanRenderer() = default;

    bool Initialize();
    void Shutdown();

    VkCommandBuffer BeginFrame();
    void EndFrame();
    // Present
    void SubmitRenderItem(const SRenderItem& renderItem);
    void SubimtRenderItems(const std::vector<SRenderItem>& vRenderItems);
    void FlushRenderItems(ICommandList* pCmd);

    void BeginSwapchainRenderPass(VkCommandBuffer commandBuffer);
    void EndSwapchainRenderPass(VkCommandBuffer commandBuffer);

    bool IsFrameInProgress() const;
    VkCommandBuffer GetCurrentCommandBuffer() const;
    uint32_t GetCurrentFrameIndex() const;
    uint32_t GetCurrentImageIndex() const;

    void OnResize();

    std::vector<SRenderItem>& GetRenderItems();
    const std::vector<SRenderItem>& GetRenderItems() const;

private:
    bool CreateCommandBuffers();
    void FreeCommandBuffers();
    bool RecreateSwapchain();

private:
    bool m_bFrameStarted = false;
    uint32_t m_uiCurrentFrame = 0;
    uint32_t m_uiCurrentImageIndex = 0;
    std::vector<VkCommandBuffer> m_vkvCommandBuffers;
    std::unique_ptr<CVulkanSyncObject> m_pVulkanSyncObject = nullptr;
    bool m_bFramebufferResized = false;
    std::vector<SRenderItem> m_vRenderItems = {};
};
