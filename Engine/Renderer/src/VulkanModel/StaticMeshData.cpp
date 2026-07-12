#include "VulkanModel/StaticMeshData.h"
#include "Vulkan/VulkanUtils.h"
#include "Device/VulkanRenderDevice.h"

VkDescriptorSetLayout CStaticMeshShaderLayout::m_vksDscriptorLayout = VK_NULL_HANDLE;

VkDescriptorSetLayout CStaticMeshShaderLayout::Get(const SVulkanContext& context)
{
    if (m_vksDscriptorLayout != VK_NULL_HANDLE)
    {
        return m_vksDscriptorLayout;
    }

	const auto& bindings = CStaticMeshShaderLayout::GetBindings();

	std::vector<VkDescriptorSetLayoutBinding> vkBindings;
	vkBindings.reserve(bindings.size());
	for (const SBindingDesc& binding : bindings)
	{
		VkDescriptorSetLayoutBinding vkBinding{};
		vkBinding.binding = binding.m_uiBinding;
		vkBinding.descriptorType = VulkanUtils::ToVkDescriptorType(binding.m_eType);
		vkBinding.descriptorCount = binding.m_uiArrayCount;
		vkBinding.stageFlags = VK_SHADER_STAGE_ALL;
		vkBindings.push_back(vkBinding);
	}

	VkDescriptorSetLayoutCreateInfo layoutInfo{};
	layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	layoutInfo.bindingCount = static_cast<uint32_t>(vkBindings.size());
	layoutInfo.pBindings = vkBindings.data();

    VkResult result = vkCreateDescriptorSetLayout(context.device, &layoutInfo, nullptr, &m_vksDscriptorLayout);
    if (result != VK_SUCCESS)
    {
        syserr("CStaticMeshShaderLayout::Get: vkCreateDescriptorSetLayout failed",);
        m_vksDscriptorLayout = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }
    return m_vksDscriptorLayout;
}

void CStaticMeshShaderLayout::Destroy(const SVulkanContext& context)
{
    if (m_vksDscriptorLayout != VK_NULL_HANDLE)
    {
        vkDestroyDescriptorSetLayout(context.device, m_vksDscriptorLayout, nullptr);
        m_vksDscriptorLayout = VK_NULL_HANDLE;
    }
}

const std::vector<SBindingDesc>& CStaticMeshShaderLayout::GetBindings()
{
    static const std::vector<SBindingDesc> bindings = {
        { 0, EBindingType::BIND_TYPE_UNIFORM_BUFFER, 1 },
        { 1, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 2, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 3, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 4, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 5, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 6, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 }
    };
    return bindings;
}