#include "Vulkan/VulkanDescriptorContext.h"
#include "Device/VulkanRenderDevice.h"
#include "Logging/LogManager.h"
#include "Vulkan/VulkanBuffer.h"
#include "Vulkan/VulkanTexture2D.h"

bool CVulkanDescriptorContext::Initialize(const SBindingContextDesc& desc)
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	auto& vkCtx = vkRenderDevice.GetContext();
	VkDevice device = vkCtx.device;

	// Clean Up any resources
	Destroy();

	if (desc.m_uiFrameCount == 0)
	{
		syserr("DescriptorContext: frame count is 0");
		return false;
	}

	if (desc.m_vBindings.empty())
	{
		syserr("DescriptorContext: no bindings");
		return false;
	}

	// Create descriptor set layout
	std::vector<VkDescriptorSetLayoutBinding> vkBindings;
	vkBindings.reserve(desc.m_vBindings.size());
	for (const SBindingDesc& binding : desc.m_vBindings)
	{
		VkDescriptorSetLayoutBinding vkBinding{};
		vkBinding.binding = binding.m_uiBinding;
		vkBinding.descriptorType = VulkanUtils::ToVkDescriptorType(binding.m_eType);
		vkBinding.descriptorCount = binding.m_uiArrayCount;
		vkBinding.stageFlags = VK_SHADER_STAGE_ALL; // better: convert engine stage flags
		vkBinding.pImmutableSamplers = nullptr;
		vkBindings.push_back(vkBinding);
		m_mBindingTypes[binding.m_uiBinding] = binding.m_eType;
	}

	VkDescriptorSetLayoutCreateInfo layoutInfo{};
	layoutInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
	layoutInfo.bindingCount = static_cast<uint32_t>(vkBindings.size());
	layoutInfo.pBindings = vkBindings.data();

	if (vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &m_vkDescriptorSetLayout) != VK_SUCCESS)
	{
		syserr("DescriptorContext: failed to create descriptor set layout");
		return false;
	}

	// Build pool sizes from binding definitions
	// pool count = descriptor count per set * number of sets
	std::unordered_map<VkDescriptorType, uint32_t> descriptorCounts;
	for (const SBindingDesc& binding : desc.m_vBindings)
	{
		VkDescriptorType type = VulkanUtils::ToVkDescriptorType(binding.m_eType);
		descriptorCounts[type] += binding.m_uiArrayCount * desc.m_uiFrameCount;
	}

	std::vector<VkDescriptorPoolSize> poolSizes;
	poolSizes.reserve(descriptorCounts.size());

	for (const auto& it : descriptorCounts)
	{
		VkDescriptorPoolSize poolSize{};
		poolSize.type = it.first;
		poolSize.descriptorCount = it.second;
		poolSizes.push_back(poolSize);
	}

	VkDescriptorPoolCreateInfo poolInfo{};
	poolInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_POOL_CREATE_INFO;
	poolInfo.flags = VK_DESCRIPTOR_POOL_CREATE_FREE_DESCRIPTOR_SET_BIT;
	poolInfo.maxSets = desc.m_uiFrameCount;
	poolInfo.poolSizeCount = static_cast<uint32_t>(poolSizes.size());
	poolInfo.pPoolSizes = poolSizes.data();

	if (vkCreateDescriptorPool(device, &poolInfo, nullptr, &m_vkDescriptorPool) != VK_SUCCESS)
	{
		syserr("DescriptorContext: failed to create descriptor pool");
		Destroy();
		return false;
	}

	// Allocate one descriptor set per frame
	m_vkvDescriptorSetLayouts.resize(desc.m_uiFrameCount, m_vkDescriptorSetLayout);
	m_vkvDescriptorSets.resize(desc.m_uiFrameCount);

	VkDescriptorSetAllocateInfo allocInfo{};
	allocInfo.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_ALLOCATE_INFO;
	allocInfo.descriptorPool = m_vkDescriptorPool;
	allocInfo.descriptorSetCount = desc.m_uiFrameCount;
	allocInfo.pSetLayouts = m_vkvDescriptorSetLayouts.data();

	if (vkAllocateDescriptorSets(device, &allocInfo, m_vkvDescriptorSets.data()) != VK_SUCCESS)
	{
		syserr("DescriptorContext: failed to allocate descriptor sets");
		Destroy();
		return false;
	}

	// 4) Write descriptors for each frame
	// Keep infos alive until vkUpdateDescriptorSets call for that frame
	for (uint32_t frameIndex = 0; frameIndex < desc.m_uiFrameCount; ++frameIndex)
	{
		std::vector<VkWriteDescriptorSet> writes;
		std::vector<VkDescriptorBufferInfo> bufferInfos;
		std::vector<VkDescriptorImageInfo> imageInfos;

		bufferInfos.reserve(desc.m_vBufferResources.size());
		imageInfos.reserve(desc.m_vImageResources.size());
		writes.reserve(desc.m_vBufferResources.size() + desc.m_vImageResources.size());

		// Buffer bindings
		for (const SBindingBufferResource& bufferRes : desc.m_vBufferResources)
		{
			if (frameIndex >= bufferRes.m_vBuffers.size() || bufferRes.m_vBuffers[frameIndex] == nullptr)
			{
				syserr("DescriptorContext: missing buffer resource for frame %u binding %u", frameIndex, bufferRes.m_uiBinding);
				Destroy();
				return false;
			}

			CVulkanBuffer* vkBuffer = dynamic_cast<CVulkanBuffer*>(bufferRes.m_vBuffers[frameIndex]);

			VkDescriptorBufferInfo& bufferInfo = bufferInfos.emplace_back();
			bufferInfo.buffer = vkBuffer->GetBuffer();
			bufferInfo.offset = 0;
			bufferInfo.range = vkBuffer->GetSize();

			VkDescriptorType type = VulkanUtils::ToVkDescriptorType(m_mBindingTypes[bufferRes.m_uiBinding]);

			VkWriteDescriptorSet& write = writes.emplace_back();
			write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			write.dstSet = m_vkvDescriptorSets[frameIndex];
			write.dstBinding = bufferRes.m_uiBinding;
			write.dstArrayElement = 0;
			write.descriptorCount = 1;
			write.descriptorType = type; // better: derive from binding table
			write.pBufferInfo = &bufferInfo;
		}

		// Image bindings
		for (const SBindingImageResource& imageRes : desc.m_vImageResources)
		{
			if (imageRes.m_pTexture == nullptr)
			{
				syserr("DescriptorContext: missing image resource for binding %u", imageRes.m_uiBinding);
				Destroy();
				return false;
			}

			CVulkanTexture2D* vkTexture = dynamic_cast<CVulkanTexture2D*>(imageRes.m_pTexture);

			VkDescriptorImageInfo& imageInfo = imageInfos.emplace_back();
			imageInfo.imageLayout = VulkanUtils::ToVkImageLayout(imageRes.m_eLayout);
			imageInfo.imageView = vkTexture->GetImageView();
			imageInfo.sampler = vkTexture->GetSampler();

			VkDescriptorType type = VulkanUtils::ToVkDescriptorType(m_mBindingTypes[imageRes.m_uiBinding]);

			VkWriteDescriptorSet& write = writes.emplace_back();
			write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
			write.dstSet = m_vkvDescriptorSets[frameIndex];
			write.dstBinding = imageRes.m_uiBinding;
			write.dstArrayElement = 0;
			write.descriptorCount = 1;
			write.descriptorType = type; // better: derive from binding table
			write.pImageInfo = &imageInfo;
		}

		vkUpdateDescriptorSets(device, static_cast<uint32_t>(writes.size()), writes.data(), 0, nullptr);
	}

	m_bInitialized = true;
	return true;
}

void CVulkanDescriptorContext::Destroy()
{
	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);

	auto& context = vkRenderDevice.GetContext();

	if (m_vkDescriptorPool != VK_NULL_HANDLE && m_vkvDescriptorSets.size() > 0)
	{
		vkFreeDescriptorSets(context.device, m_vkDescriptorPool, m_vkvDescriptorSets.size(), m_vkvDescriptorSets.data());
		m_vkvDescriptorSets.clear();
	}

	if (m_vkDescriptorSetLayout != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorSetLayout(context.device, m_vkDescriptorSetLayout, nullptr);
		m_vkDescriptorSetLayout = VK_NULL_HANDLE;
	}

	m_vkvDescriptorSets.clear();

	if (m_vkDescriptorPool != VK_NULL_HANDLE)
	{
		vkDestroyDescriptorPool(context.device, m_vkDescriptorPool, nullptr);
		m_vkDescriptorPool = VK_NULL_HANDLE;
	}

	m_mBindingTypes.clear();
}

VkDescriptorSetLayout CVulkanDescriptorContext::CreateLayoutFromBindings(VkDevice device, const std::vector<SBindingDesc>& bindings)
{
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

	VkDescriptorSetLayout layout = VK_NULL_HANDLE;
	vkCreateDescriptorSetLayout(device, &layoutInfo, nullptr, &layout);
	return layout;
}

VkDescriptorSet CVulkanDescriptorContext::GetDescriptorSet(uint32_t frameIndex)
{
	if (frameIndex >= m_vkvDescriptorSets.size())
	{
		return VK_NULL_HANDLE;
	}

	return (m_vkvDescriptorSets.at(frameIndex));
}

const std::vector<VkDescriptorSet>& CVulkanDescriptorContext::GetDescriptorSets() const
{
	return (m_vkvDescriptorSets);
}

VkDescriptorSetLayout CVulkanDescriptorContext::GetDescriptorSetLayout() const
{
	return (m_vkDescriptorSetLayout);
}

bool CVulkanDescriptorContext::UpdateBufferBinding(
	uint32_t frameIndex,
	uint32_t binding,
	IBuffer* pBuffer,
	VkDeviceSize offset,
	VkDeviceSize range)
{
	if (!pBuffer)
	{
		return false;
	}

	if (frameIndex >= m_vkvDescriptorSets.size())
	{
		return false;
	}

	auto it = m_mBindingTypes.find(binding);
	if (it == m_mBindingTypes.end())
	{
		return false;
	}

	auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
	auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
	VkDevice device = vkRenderDevice.GetContext().device;

	CVulkanBuffer* vkBuffer = dynamic_cast<CVulkanBuffer*>(pBuffer);

	if (!vkBuffer)
	{
		return false;
	}

	VkDescriptorBufferInfo bufferInfo{};
	bufferInfo.buffer = vkBuffer->GetBuffer();
	bufferInfo.offset = offset;
	bufferInfo.range = range;

	VkWriteDescriptorSet write{};
	write.sType = VK_STRUCTURE_TYPE_WRITE_DESCRIPTOR_SET;
	write.dstSet = m_vkvDescriptorSets[frameIndex];
	write.dstBinding = binding;
	write.dstArrayElement = 0;
	write.descriptorCount = 1;
	write.descriptorType = VulkanUtils::ToVkDescriptorType(it->second);
	write.pBufferInfo = &bufferInfo;

	vkUpdateDescriptorSets(device, 1, &write, 0, nullptr);
	return true;
}