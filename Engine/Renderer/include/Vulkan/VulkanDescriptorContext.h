#pragma once

#include "API/BindingContext.h"

class CVulkanDescriptorContext : public IBindingContext
{
public:
	CVulkanDescriptorContext() = default;
	~CVulkanDescriptorContext() override = default;

	bool Initialize(const SBindingContextDesc& desc) override;
	void Destroy() override;

	VkDescriptorSetLayout CreateLayoutFromBindings(VkDevice device, const std::vector<SBindingDesc>& bindings);
	VkDescriptorSet GetDescriptorSet(uint32_t frameIndex);
	const std::vector<VkDescriptorSet>& GetDescriptorSets() const;
	VkDescriptorSetLayout GetDescriptorSetLayout() const;
	bool UpdateBufferBinding(uint32_t frameIndex, uint32_t binding, IBuffer* pBuffer, VkDeviceSize offset, VkDeviceSize range);

private:
	VkDescriptorSetLayout m_vkDescriptorSetLayout = VK_NULL_HANDLE;
	VkDescriptorPool m_vkDescriptorPool = VK_NULL_HANDLE;
	std::vector<VkDescriptorSet> m_vkvDescriptorSets = {};
	std::vector<VkDescriptorSetLayout> m_vkvDescriptorSetLayouts = {};
	std::unordered_map<uint32_t, EBindingType> m_mBindingTypes;
	bool m_bInitialized = false;
};