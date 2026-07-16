#pragma once

#include <vulkan/vulkan.h>
#include <vector>
#include "API/ActorData.h"
#include "API/BindingContext.h"
#include "Transform.h"
#include "BoundingBox.h"

struct SVulkanContext;

class CDescriptorSetLayouts
{
public:
    static VkDescriptorSetLayout GetMaterialsDescriptorsetLayout(const SVulkanContext& context); // built once, lazily, cached statically
    static VkDescriptorSetLayout GetFrameDescriptorsetLayout(const SVulkanContext& context);
    static void Destroy(const SVulkanContext& context);
    static const std::vector<SBindingDesc>& GetMaterialsBindings();
    static const std::vector<SBindingDesc>& GetFrameBindings();
private:
    static VkDescriptorSetLayout m_vkDescriptorSetLayoutMaterials;
    static VkDescriptorSetLayout m_vkDescriptorSetLayoutFrame;
};
