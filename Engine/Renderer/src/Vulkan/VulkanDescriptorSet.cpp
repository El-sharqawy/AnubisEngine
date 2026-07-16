#include "Vulkan/VulkanDescriptorSet.h"
#include "Vulkan/VulkanUtils.h"
#include "Device/VulkanRenderDevice.h"

VkDescriptorSetLayout CDescriptorSetLayouts::m_vkDescriptorSetLayoutMaterials = VK_NULL_HANDLE;
VkDescriptorSetLayout CDescriptorSetLayouts::m_vkDescriptorSetLayoutFrame = VK_NULL_HANDLE;

VkDescriptorSetLayout CDescriptorSetLayouts::GetMaterialsDescriptorsetLayout(const SVulkanContext& context)
{
    if (m_vkDescriptorSetLayoutMaterials != VK_NULL_HANDLE)
    {
        return m_vkDescriptorSetLayoutMaterials;
    }

	const auto& bindings = CDescriptorSetLayouts::GetMaterialsBindings();

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

    VkResult result = vkCreateDescriptorSetLayout(context.device, &layoutInfo, nullptr, &m_vkDescriptorSetLayoutMaterials);
    if (result != VK_SUCCESS)
    {
        syserr("vkCreateDescriptorSetLayout for m_vkDescriptorSetLayoutMaterials failed");
        m_vkDescriptorSetLayoutMaterials = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }
    return m_vkDescriptorSetLayoutMaterials;
}

VkDescriptorSetLayout CDescriptorSetLayouts::GetFrameDescriptorsetLayout(const SVulkanContext& context)
{
    if (m_vkDescriptorSetLayoutFrame != VK_NULL_HANDLE)
    {
        return m_vkDescriptorSetLayoutFrame;
    }

    const auto& bindings = CDescriptorSetLayouts::GetFrameBindings();

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

    VkResult result = vkCreateDescriptorSetLayout(context.device, &layoutInfo, nullptr, &m_vkDescriptorSetLayoutFrame);
    if (result != VK_SUCCESS)
    {
        syserr("vkCreateDescriptorSetLayout for m_vkDescriptorSetLayoutFrame failed");
        m_vkDescriptorSetLayoutFrame = VK_NULL_HANDLE;
        return VK_NULL_HANDLE;
    }
    return m_vkDescriptorSetLayoutFrame;
}

void CDescriptorSetLayouts::Destroy(const SVulkanContext& context)
{
    if (m_vkDescriptorSetLayoutMaterials != VK_NULL_HANDLE)
    {
        vkDestroyDescriptorSetLayout(context.device, m_vkDescriptorSetLayoutMaterials, nullptr);
        m_vkDescriptorSetLayoutMaterials = VK_NULL_HANDLE;
    }

    if (m_vkDescriptorSetLayoutFrame != VK_NULL_HANDLE)
    {
        vkDestroyDescriptorSetLayout(context.device, m_vkDescriptorSetLayoutFrame, nullptr);
        m_vkDescriptorSetLayoutFrame = VK_NULL_HANDLE;
    }
}

const std::vector<SBindingDesc>& CDescriptorSetLayouts::GetMaterialsBindings()
{
    static const std::vector<SBindingDesc> s_MaterialsBindings = {
        { 0, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 1, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 2, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 3, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 4, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
        { 5, EBindingType::BIND_TYPE_COMBINED_IMAGE_SAMPLER, 1 },
    };

    return (s_MaterialsBindings);
}

const std::vector<SBindingDesc>& CDescriptorSetLayouts::GetFrameBindings()
{
    static const std::vector<SBindingDesc> s_FrameBindings = {
        { 0, EBindingType::BIND_TYPE_UNIFORM_BUFFER, 1 },
        { 1, EBindingType::BIND_TYPE_STORAGE_BUFFER, 1 },
    };

    return (s_FrameBindings);
}
