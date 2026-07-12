#pragma once

#include "TypeVector2.h"
#include "TypeVector3.h"
#include "TypeVector4.h"
#include "API/BindingContext.h"
#include <vulkan/vulkan.h>

struct SVulkanContext;

struct SStaticMeshVertex
{
    SVector3Df position;   // xyz
    SVector3Df normal;     // xyz
    SVector2Df texCoord;   // uv
    SVector4Df tangent;    // xyz = tangent, w = handedness (+1 / -1)

    SStaticMeshVertex() = default;

    SStaticMeshVertex(
        const SVector3Df& inPosition,
        const SVector3Df& inNormal,
        const SVector2Df& inTexCoord,
        const SVector4Df& inTangent = SVector4Df(1.f, 0.f, 0.f, 1.f))
        : position(inPosition)
        , normal(inNormal)
        , texCoord(inTexCoord)
        , tangent(inTangent)
    {
    }

    static VkVertexInputBindingDescription GetBindingDescription()
    {
        VkVertexInputBindingDescription bindingDescription{};
        bindingDescription.binding = 0;
        bindingDescription.stride = sizeof(SStaticMeshVertex);
        bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
        return bindingDescription;
    }

    static std::vector<VkVertexInputAttributeDescription> GetAttributeDescriptions()
    {
        std::vector<VkVertexInputAttributeDescription> attributeDescriptions{};
        attributeDescriptions.resize(4);

        attributeDescriptions[0].binding = 0;
        attributeDescriptions[0].location = 0;
        attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[0].offset = offsetof(SStaticMeshVertex, position);

        attributeDescriptions[1].binding = 0;
        attributeDescriptions[1].location = 1;
        attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
        attributeDescriptions[1].offset = offsetof(SStaticMeshVertex, normal);

        attributeDescriptions[2].binding = 0;
        attributeDescriptions[2].location = 2;
        attributeDescriptions[2].format = VK_FORMAT_R32G32_SFLOAT;
        attributeDescriptions[2].offset = offsetof(SStaticMeshVertex, texCoord);

        attributeDescriptions[3].binding = 0;
        attributeDescriptions[3].location = 3;
        attributeDescriptions[3].format = VK_FORMAT_R32G32B32A32_SFLOAT;
        attributeDescriptions[3].offset = offsetof(SStaticMeshVertex, tangent);

        return attributeDescriptions;
    }
};

class CStaticMeshShaderLayout
{
public:
    static VkDescriptorSetLayout Get(const SVulkanContext& context); // built once, lazily, cached statically
    static const std::vector<SBindingDesc>& GetBindings();
    static void Destroy(const SVulkanContext& context);

private:
    static VkDescriptorSetLayout m_vksDscriptorLayout;
};
