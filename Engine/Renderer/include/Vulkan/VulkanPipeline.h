#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include <string>
#include "API/Pipeline.h"
#include "CoreTypes.h"
#include "Vulkan/VulkanShader.h"

class CVulkanShader;
struct SVulkanContext;

class CVulkanPipeline : public IPipeline
{
public:
	CVulkanPipeline() = default;
	~CVulkanPipeline() = default;

	bool Initialize(const SPipelineDesc& desc, VkRenderPass vkRenderPass, VkExtent2D extent);
	void Clear();

	VkRenderPass GetRenderPass() const;
	VkPipeline GetGraphicsPipeline() const;
	VkPipelineLayout GetPipelineLayout() const;
	EPipelineType GetPipelineType() const override;

	bool CreatePipelineLayout(const SVulkanContext& context, const std::vector<VkDescriptorSetLayout>& setLayouts);

protected:
	bool CreateShaderStages(const SVulkanContext& context, const SPipelineDesc& desc, std::array<VkPipelineShaderStageCreateInfo, 2>& shaderStages);
	void BuildVertexInput(const SPipelineDesc& desc, VkPipelineVertexInputStateCreateInfo& vertexInputInfo, VkVertexInputBindingDescription& bindingDesc, std::vector<VkVertexInputAttributeDescription>& attrDescs);
	void BuildInputAssembly(const SPipelineDesc& desc, VkPipelineInputAssemblyStateCreateInfo& inputAssemblyInfo);
	void BuildViewportState(const VkExtent2D& extent, VkPipelineViewportStateCreateInfo& viewportState);
	void BuildRasterizer(const SPipelineDesc& desc, VkPipelineRasterizationStateCreateInfo& rasterInfo);
	void BuildMultisampling(VkPipelineMultisampleStateCreateInfo& msaaInfo);
	void BuildDepthStencil(const SPipelineDesc& desc, VkPipelineDepthStencilStateCreateInfo& depthInfo);
	void BuildColorBlend(const SPipelineDesc& desc, VkPipelineColorBlendAttachmentState& blendAttachment, VkPipelineColorBlendStateCreateInfo& blendInfo);

private:
	VkPipelineLayout m_vkPipelineLayout = VK_NULL_HANDLE;
	VkPipeline m_vkGraphicsPipeline = VK_NULL_HANDLE;
	std::unique_ptr<CVulkanShader> m_pPipelineShader = nullptr;
	std::string m_stPipelineName;
};