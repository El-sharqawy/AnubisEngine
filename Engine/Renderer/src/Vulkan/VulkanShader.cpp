#include "Vulkan/VulkanShader.h"
#include "Vulkan/VulkanUtils.h"
#include "Vulkan/VulkanDevice.h"
#include "Device/VulkanRenderDevice.h"

bool CVulkanShader::Create(const SShaderDesc& desc)
{
	SVulkanContext& context = GetVulkanContext();

	for (const auto& stageDesc : desc.m_vStages)
	{
		std::vector<uint32_t> shaderCode = VulkanUtils::ReadShaderData(stageDesc.path);

		if (shaderCode.empty())
		{
			syserr("Failed to compile stage for shader '{}': {}", desc.m_stName, stageDesc.path);
			return false;
		}

		VkShaderModuleCreateInfo moduleInfo{};
		moduleInfo.sType = VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO;
		moduleInfo.pNext = nullptr;
		moduleInfo.flags = 0;
		moduleInfo.codeSize = shaderCode.size() * sizeof(uint32_t);
		moduleInfo.pCode = shaderCode.data();

		VkShaderModule module = VK_NULL_HANDLE;
		if (VK_CHECK_BOOL(vkCreateShaderModule(context.device, &moduleInfo, nullptr, &module)) == false)
		{
			syserr("Failed to create shader module for '{}'", stageDesc.path);
			Destroy(); // clean up any modules created so far
			return false;
		}

		m_vModules.push_back({ stageDesc.stageType, module });

		VkPipelineShaderStageCreateInfo stageInfo{};
		stageInfo.sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
		stageInfo.stage = VulkanUtils::ToVkShaderInfo(stageDesc.stageType);
		stageInfo.module = module;
		stageInfo.pName = "main";
		m_vStageInfos.push_back(stageInfo);
	}

	return (true);
}

void CVulkanShader::Destroy()
{
	auto& context = static_cast<CVulkanRenderDevice&>(CServiceLocator::Get<CIRenderDevice>()).GetContext();

	for (auto& shaderModule : m_vModules)
	{
		if (shaderModule.module != VK_NULL_HANDLE)
		{
			vkDestroyShaderModule(context.device, shaderModule.module, nullptr);
		}
	}

	m_vModules.clear();
	m_vStageInfos.clear();
}

bool CVulkanShader::HasStage(EShaderStage stage) const
{
	return std::any_of(m_vModules.begin(), m_vModules.end(), [stage](const SStageModule& s) { return s.stage == stage; });
}

bool CVulkanShader::IsCompute() const
{
	const EShaderStage stage = EShaderStage::SHADER_TYPE_COMPUTE;
	return std::any_of(m_vModules.begin(), m_vModules.end(), [stage](const SStageModule& s) { return s.stage == stage; });
}

std::vector<VkPipelineShaderStageCreateInfo>& CVulkanShader::GetShaderStagesInfo()
{
	return (m_vStageInfos);
}

const std::vector<VkPipelineShaderStageCreateInfo>& CVulkanShader::GetShaderStagesInfo() const
{
	return (m_vStageInfos);
}

SVulkanContext& CVulkanShader::GetVulkanContext() const
{
	return static_cast<CVulkanRenderDevice&>(CServiceLocator::Get<CIRenderDevice>()).GetContext();
}
