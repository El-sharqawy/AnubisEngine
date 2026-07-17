#pragma once

#include "API/CommandList.h"
#include <vulkan/vulkan.h>

class CVulkanRenderer;
class CVulkanPipeline;

class CVulkanCommandList : public ICommandList
{
public:
    CVulkanCommandList() = default;
    ~CVulkanCommandList() = default;

    void SetRenderer(CVulkanRenderer* pRenderer);

    void Begin() override;
    void End() override;

    void SetViewport(const SViewport& vp) override;
    void SetScissor(const SRect2D& rect) override;

    void BindPipeline(IPipeline* pipeline) override;
    void BindVertexBuffer(IBuffer* buffer, uint64_t offset = 0) override;
    void BindIndexBuffer(IBuffer* buffer, EIndexType type, uint64_t offset = 0) override;
    void BindMaterial(IMaterial* material, uint32_t frameIndex) override;
    void BindFrameDescriptorSet(VkDescriptorSet set, EBiningLayoutSetsPoints firstSet);
    void PushConstants(const void* data, uint32_t size, uint32_t offset = 0) override;

    void DrawIndexed(uint32_t indexCount,
        uint32_t instanceCount = 1,
        uint32_t firstIndex = 0,
        int32_t vertexOffset = 0,
        uint32_t firstInstance = 0) override;

private:
    VkCommandBuffer GetActiveCommandBuffer() const;

private:
    CVulkanRenderer* m_pRenderer = nullptr;
    CVulkanPipeline* m_pBoundPipeline = nullptr;
};
