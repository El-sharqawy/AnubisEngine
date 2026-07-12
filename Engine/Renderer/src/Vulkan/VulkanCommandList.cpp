#include "Vulkan/VulkanCommandList.h"
#include "Vulkan/VUlkanRenderer.h"
#include "Vulkan/VulkanPipeline.h"
#include "VUlkan/VulkanUtils.h"
#include "Vulkan/VulkanMaterial.h"
#include "Vulkan/VulkanBuffer.h"

void CVulkanCommandList::SetRenderer(CVulkanRenderer* pRenderer)
{
	m_pRenderer = pRenderer;
}

void CVulkanCommandList::Begin()
{
	assert(m_pRenderer != nullptr);
	assert(m_pRenderer->IsFrameInProgress());
}

void CVulkanCommandList::End()
{
	assert(m_pRenderer != nullptr);
	assert(m_pRenderer->IsFrameInProgress());
}

void CVulkanCommandList::SetViewport(const SViewport& vp)
{
	VkViewport viewport{};
	viewport.x = vp.x;
	viewport.y = vp.y;
	viewport.width = vp.width;
	viewport.height = vp.height;
	viewport.minDepth = vp.minDepth;
	viewport.maxDepth = vp.maxDepth;

	vkCmdSetViewport(GetActiveCommandBuffer(), 0, 1, &viewport);
}

void CVulkanCommandList::SetScissor(const SRect2D& rect)
{
	VkRect2D scissor{};
	scissor.offset = { rect.x, rect.y };
	scissor.extent = { rect.width, rect.height };

	vkCmdSetScissor(GetActiveCommandBuffer(), 0, 1, &scissor);
}

void CVulkanCommandList::BindPipeline(IPipeline* pipeline)
{
	assert(pipeline != nullptr);
	auto* vkPipeline = static_cast<CVulkanPipeline*>(pipeline);
	VkCommandBuffer cmd = GetActiveCommandBuffer();

	vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, vkPipeline->GetGraphicsPipeline());
	m_pBoundPipeline = vkPipeline;
}

void CVulkanCommandList::BindVertexBuffer(IBuffer* buffer, uint64_t offset)
{
	assert(buffer != nullptr);

	auto* vkBuffer = static_cast<CVulkanBuffer*>(buffer);
	VkCommandBuffer cmd = GetActiveCommandBuffer();

	VkBuffer buffers[] = { vkBuffer->GetBuffer() };
	VkDeviceSize offsets[] = { static_cast<VkDeviceSize>(offset) };

	vkCmdBindVertexBuffers(cmd, 0, 1, buffers, offsets);
}

void CVulkanCommandList::BindIndexBuffer(IBuffer* buffer, EIndexType type, uint64_t offset)
{
	assert(buffer != nullptr);

	auto* vkBuffer = static_cast<CVulkanBuffer*>(buffer);
	VkCommandBuffer cmd = GetActiveCommandBuffer();

	VkIndexType vkIndexType = VulkanUtils::ToVkIndexType(type);

	vkCmdBindIndexBuffer(cmd, vkBuffer->GetBuffer(), static_cast<VkDeviceSize>(offset), vkIndexType);
}

void CVulkanCommandList::BindMaterial(IMaterial* material, uint32_t frameIndex)
{
	assert(material != nullptr);
	assert(m_pBoundPipeline != nullptr);

	auto* vkMaterial = static_cast<CVulkanMaterial*>(material);
	VkCommandBuffer cmd = GetActiveCommandBuffer();

	VkDescriptorSet set = vkMaterial->GetDescriptorSet(frameIndex);

	vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, m_pBoundPipeline->GetPipelineLayout(), 0, 1, &set, 0, nullptr);
}

void CVulkanCommandList::PushConstants(const void* data, uint32_t size, uint32_t offset)
{
	assert(data != nullptr);
	assert(m_pBoundPipeline != nullptr);

	VkCommandBuffer cmd = GetActiveCommandBuffer();

	vkCmdPushConstants(cmd, m_pBoundPipeline->GetPipelineLayout(), VK_SHADER_STAGE_VERTEX_BIT, offset, size, data);

}

void CVulkanCommandList::DrawIndexed(uint32_t indexCount, uint32_t instanceCount, uint32_t firstIndex, int32_t vertexOffset, uint32_t firstInstance)
{
	VkCommandBuffer cmd = GetActiveCommandBuffer();
	vkCmdDrawIndexed(cmd, indexCount, instanceCount, firstIndex, vertexOffset, firstInstance);
}

VkCommandBuffer CVulkanCommandList::GetActiveCommandBuffer() const
{
	assert(m_pRenderer != nullptr);
	assert(m_pRenderer->IsFrameInProgress());
	return m_pRenderer->GetCurrentCommandBuffer();
}
