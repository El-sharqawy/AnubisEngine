#include "Vulkan/VulkanPipeline.h"
#include "Vulkan/VulkanUtils.h"
#include "Vulkan/VulkanDevice.h"
#include "Vulkan/VulkanSwapchain.h"
#include "Device/VulkanRenderDevice.h"

bool CVulkanPipeline::Initialize(const SPipelineDesc& desc, VkRenderPass vkRenderPass, VkExtent2D extent)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& context = vkRenderDevice.GetContext();

	Clear();

	// must be called early
	if (!CreatePipelineLayout(context, desc.descriptorSetLayouts))
	{
		return false;
	}

	std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages{};
	if (!CreateShaderStages(context, desc, shaderStages))
	{
		return false;
	}

	VkPipelineVertexInputStateCreateInfo vertexInputInfo{};
	VkVertexInputBindingDescription bindingDesc{};
	std::vector<VkVertexInputAttributeDescription> attrDescs;
	BuildVertexInput(desc, vertexInputInfo, bindingDesc, attrDescs);

	VkPipelineInputAssemblyStateCreateInfo inputAssemblyInfo{};
	BuildInputAssembly(desc, inputAssemblyInfo);

	VkPipelineViewportStateCreateInfo viewportState{};
	BuildViewportState(extent, viewportState);

	VkPipelineRasterizationStateCreateInfo rasterInfo{};
	BuildRasterizer(desc, rasterInfo);

	VkPipelineMultisampleStateCreateInfo msaaInfo{};
	BuildMultisampling(msaaInfo);

	VkPipelineDepthStencilStateCreateInfo depthInfo{};
	BuildDepthStencil(desc, depthInfo);

	VkPipelineColorBlendAttachmentState blendAttachment{};
	VkPipelineColorBlendStateCreateInfo blendInfo{};
	BuildColorBlend(desc, blendAttachment, blendInfo);

	std::array<VkDynamicState, 2> dynamicStates = {
		VK_DYNAMIC_STATE_VIEWPORT,
		VK_DYNAMIC_STATE_SCISSOR
	};

	// Dynamic States
	VkPipelineDynamicStateCreateInfo dynaimcStateCreateInfo{};
	dynaimcStateCreateInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO;
	dynaimcStateCreateInfo.pNext = nullptr;
	dynaimcStateCreateInfo.flags = 0;
	dynaimcStateCreateInfo.dynamicStateCount = static_cast<uint32_t>(dynamicStates.size());
	dynaimcStateCreateInfo.pDynamicStates = dynamicStates.data();

	VkGraphicsPipelineCreateInfo pipelineCreateInfo{};
	pipelineCreateInfo.sType = VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO;
	pipelineCreateInfo.pNext = nullptr;
	pipelineCreateInfo.flags = 0;
	pipelineCreateInfo.stageCount = 2;
	pipelineCreateInfo.pStages = shaderStages.data();
	pipelineCreateInfo.pVertexInputState = &vertexInputInfo;
	pipelineCreateInfo.pInputAssemblyState = &inputAssemblyInfo;
	pipelineCreateInfo.pTessellationState = nullptr;
	pipelineCreateInfo.pViewportState = &viewportState;
	pipelineCreateInfo.pRasterizationState = &rasterInfo;
	pipelineCreateInfo.pMultisampleState = &msaaInfo;
	pipelineCreateInfo.pDepthStencilState = &depthInfo;
	pipelineCreateInfo.pColorBlendState = &blendInfo;
	pipelineCreateInfo.pDynamicState = &dynaimcStateCreateInfo;
	pipelineCreateInfo.layout = m_vkPipelineLayout;
	pipelineCreateInfo.renderPass = vkRenderPass;
	pipelineCreateInfo.subpass = 0;
	pipelineCreateInfo.basePipelineHandle = VK_NULL_HANDLE;
	pipelineCreateInfo.basePipelineIndex = -1;

	if (vkCreateGraphicsPipelines(context.device, VK_NULL_HANDLE, 1, &pipelineCreateInfo, VK_NULL_HANDLE, &m_vkGraphicsPipeline) != VK_SUCCESS)
	{
		syserr("Failed to Create Graphics Pipeline");
		return (false);
	}

	// Cleanup resources (Shaders)
	m_pPipelineShader->Destroy();

	return (true);
}

void CVulkanPipeline::Clear()
{
	m_vkGraphicsPipeline = VK_NULL_HANDLE;
	m_vkPipelineLayout = VK_NULL_HANDLE;
}

VkPipeline CVulkanPipeline::GetGraphicsPipeline() const
{
	return (m_vkGraphicsPipeline);
}

VkPipelineLayout CVulkanPipeline::GetPipelineLayout() const
{
	return (m_vkPipelineLayout);
}

EPipelineType CVulkanPipeline::GetPipelineType() const
{
	return (m_ePipelineType);
}

bool CVulkanPipeline::CreatePipelineLayout(const SVulkanContext& context, const std::vector<VkDescriptorSetLayout>& setLayouts)
{
	VkPushConstantRange pushConstantRange{};
	pushConstantRange.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
	pushConstantRange.offset = 0;
	pushConstantRange.size = sizeof(SPushConstantModel);

	VkPipelineLayoutCreateInfo createInfo{};
	createInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
	createInfo.pNext = nullptr;
	createInfo.flags = 0;
	createInfo.setLayoutCount = static_cast<uint32_t>(setLayouts.size());
	createInfo.pSetLayouts = setLayouts.empty() ? nullptr : setLayouts.data();
	createInfo.pushConstantRangeCount = 1;
	createInfo.pPushConstantRanges = &pushConstantRange;

	if (vkCreatePipelineLayout(context.device, &createInfo, nullptr, &m_vkPipelineLayout) != VK_SUCCESS)
	{
		syserr("Failed to create pipeline layout");
		return false;
	}

	return (true);
}

bool CVulkanPipeline::CreateShaderStages(const SVulkanContext& context, const SPipelineDesc& desc, std::array<VkPipelineShaderStageCreateInfo, 2>& shaderStages)
{
	SShaderDesc shaderDesc = desc.shader;

	m_pPipelineShader = std::make_unique<CVulkanShader>();
	if (!m_pPipelineShader->Create(shaderDesc))
	{
		return (false);
	}

	shaderStages[0] = m_pPipelineShader->GetShaderStagesInfo().at(0);
	shaderStages[1] = m_pPipelineShader->GetShaderStagesInfo().at(1);
	return (true);
}

void CVulkanPipeline::BuildVertexInput(const SPipelineDesc& desc, VkPipelineVertexInputStateCreateInfo& vertexInputInfo, VkVertexInputBindingDescription& bindingDesc, std::vector<VkVertexInputAttributeDescription>& attrDescs)
{
	vertexInputInfo = {};
	vertexInputInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO;
	vertexInputInfo.pNext = nullptr;
	vertexInputInfo.flags = 0;
	attrDescs = desc.vertexAttributeDescriptions;

	if (desc.hasVertexInput)
	{
		bindingDesc = desc.vertexBindingDescription;
		vertexInputInfo.vertexBindingDescriptionCount = 1;
		vertexInputInfo.pVertexBindingDescriptions = &bindingDesc;
		vertexInputInfo.vertexAttributeDescriptionCount = static_cast<uint32_t>(attrDescs.size());
		vertexInputInfo.pVertexAttributeDescriptions = attrDescs.empty() ? nullptr : attrDescs.data();
	}
	else
	{
		vertexInputInfo.vertexBindingDescriptionCount = 0;
		vertexInputInfo.pVertexBindingDescriptions = nullptr;
		vertexInputInfo.vertexAttributeDescriptionCount = 0;
		vertexInputInfo.pVertexAttributeDescriptions = nullptr;
	}

}

void CVulkanPipeline::BuildInputAssembly(const SPipelineDesc& desc, VkPipelineInputAssemblyStateCreateInfo& inputAssemblyInfo)
{
	// Input Assembly
	inputAssemblyInfo = {};
	inputAssemblyInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO;
	inputAssemblyInfo.pNext = nullptr;
	inputAssemblyInfo.flags = 0;
	inputAssemblyInfo.topology = VulkanUtils::ToVkTopology(desc.topology);
	inputAssemblyInfo.primitiveRestartEnable = VK_FALSE;
}

void CVulkanPipeline::BuildViewportState(const VkExtent2D& extent, VkPipelineViewportStateCreateInfo& viewportState)
{
	VkViewport viewport{};
	viewport.x = 0.0f;
	viewport.y = 0.0f;
	viewport.width = static_cast<float>(extent.width);
	viewport.height = static_cast<float>(extent.height);
	viewport.minDepth = 0.0f;
	viewport.maxDepth = 1.0f;

	VkRect2D scissor{};
	scissor.offset = { 0, 0 };
	scissor.extent = extent;

	viewportState = {};
	viewportState.sType = VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO;
	viewportState.pNext = nullptr;
	viewportState.flags = 0;
	viewportState.viewportCount = 1;
	viewportState.pViewports = &viewport;
	viewportState.scissorCount = 1;
	viewportState.pScissors = &scissor;
}

void CVulkanPipeline::BuildRasterizer(const SPipelineDesc& desc, VkPipelineRasterizationStateCreateInfo& rasterInfo)
{
	// Rasterizer
	rasterInfo = {};
	rasterInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO;
	rasterInfo.pNext = nullptr;
	rasterInfo.flags = 0;
	rasterInfo.depthClampEnable = VK_FALSE;
	rasterInfo.rasterizerDiscardEnable = VK_FALSE;
	rasterInfo.polygonMode = VulkanUtils::ToVkPolygonMode(desc.polygonMode);
	rasterInfo.cullMode = VulkanUtils::ToVkCullMode(desc.cullMode);
	rasterInfo.frontFace = VulkanUtils::ToVkFrontFace(desc.frontFace);
	rasterInfo.depthBiasEnable = VK_FALSE;
	rasterInfo.depthBiasConstantFactor = 0.0f;
	rasterInfo.depthBiasClamp = 0.0f;
	rasterInfo.depthBiasSlopeFactor = 0.0f;
	rasterInfo.lineWidth = 1.0f;
}

void CVulkanPipeline::BuildMultisampling(VkPipelineMultisampleStateCreateInfo& msaaInfo)
{
	// Multisampling
	msaaInfo = {};
	msaaInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO;
	msaaInfo.pNext = nullptr;
	msaaInfo.flags = 0;
	msaaInfo.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
	msaaInfo.sampleShadingEnable = VK_FALSE;
	msaaInfo.minSampleShading = 1.0f;
	msaaInfo.pSampleMask = nullptr;
	msaaInfo.alphaToCoverageEnable = VK_FALSE;
	msaaInfo.alphaToOneEnable = VK_FALSE;
}

void CVulkanPipeline::BuildDepthStencil(const SPipelineDesc& desc, VkPipelineDepthStencilStateCreateInfo& depthInfo)
{
	depthInfo = {};
	depthInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_DEPTH_STENCIL_STATE_CREATE_INFO;
	depthInfo.pNext = nullptr;
	depthInfo.flags = 0;
	depthInfo.depthTestEnable = desc.depthTest ? VK_TRUE : VK_FALSE;
	depthInfo.depthWriteEnable = desc.depthWrite ? VK_TRUE : VK_FALSE;
	depthInfo.depthCompareOp = VulkanUtils::ToVkCompareOp(desc.depthCompareOp); // lower depth = closer
	depthInfo.depthBoundsTestEnable = VK_FALSE;
	depthInfo.stencilTestEnable = VK_FALSE;
	depthInfo.front = {};
	depthInfo.back = {};
	depthInfo.minDepthBounds = 0.0f;
	depthInfo.maxDepthBounds = 1.0f;
}

void CVulkanPipeline::BuildColorBlend(const SPipelineDesc& desc, VkPipelineColorBlendAttachmentState& blendAttachment, VkPipelineColorBlendStateCreateInfo& blendInfo)
{
	blendAttachment = {};
	if (desc.blending)
	{
		blendAttachment.blendEnable = VK_TRUE;
		blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA;
		blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
		blendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
		blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
		blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
		blendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
	}
	else
	{
		blendAttachment.blendEnable = VK_FALSE;
		blendAttachment.srcColorBlendFactor = VK_BLEND_FACTOR_ONE;
		blendAttachment.dstColorBlendFactor = VK_BLEND_FACTOR_ZERO;
		blendAttachment.colorBlendOp = VK_BLEND_OP_ADD;
		blendAttachment.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE;
		blendAttachment.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO;
		blendAttachment.alphaBlendOp = VK_BLEND_OP_ADD;
	}

	blendAttachment.colorWriteMask = VK_COLOR_COMPONENT_R_BIT | VK_COLOR_COMPONENT_G_BIT | VK_COLOR_COMPONENT_B_BIT | VK_COLOR_COMPONENT_A_BIT;

	blendInfo = {};
	blendInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO;
	blendInfo.pNext = nullptr;
	blendInfo.flags = 0;
	blendInfo.logicOpEnable = VK_FALSE;
	blendInfo.logicOp = VK_LOGIC_OP_COPY;
	blendInfo.attachmentCount = 1;
	blendInfo.pAttachments = &blendAttachment;
	blendInfo.blendConstants[0] = 0.0f;
	blendInfo.blendConstants[1] = 0.0f;
	blendInfo.blendConstants[2] = 0.0f;
	blendInfo.blendConstants[3] = 0.0f;
}
