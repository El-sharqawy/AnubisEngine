#pragma once

#include "API/ShaderProgram.h"

enum class EPipelineType
{
    PIPELINE_TYPE_STATIC_MESH,
    PIPELINE_TYPE_SKELETAL_MESH,
};

struct SVertexBindingDesc
{
    uint32_t stride = 0;
};

struct SVertexAttributeDesc
{
    uint32_t location = 0;
    uint32_t offset = 0;
    uint32_t size = 0;
};

struct GLStateCache
{
    bool depthTest = false;
    bool cullFace = false;
    GLboolean depthWrite = GL_TRUE;
    GLenum depthFunc = GL_LESS;
    GLenum polygonMode = GL_FILL;
    GLenum cullMode = GL_BACK;
    GLenum frontFace = GL_CCW;
};

struct SPipelineDesc
{
    EPipelineType pipelineType;
    SShaderDesc shader;
    SVertexBindingDesc binding;
    std::vector<SVertexAttributeDesc> attributes;
    bool hasVertexInput = true;

    // Vulkan Only
    VkVertexInputBindingDescription vertexBindingDescription{};
    std::vector<VkVertexInputAttributeDescription> vertexAttributeDescriptions;
    std::vector<VkDescriptorSetLayout> descriptorSetLayouts;

    EPrimitiveTopology topology = EPrimitiveTopology::TOPOLOGY_TRIANGLES_LIST;
    ECullMode cullMode = ECullMode::CULL_MODE_BACK;
    EFrontFace frontFace = EFrontFace::FRONT_FACE_COUNTER_CLOCKWISE;
    EPolygonMode polygonMode = EPolygonMode::POLYGON_MODE_FILL;
    EDepthCompareOp depthCompareOp = EDepthCompareOp::DEPTH_LESS;

    bool cullFace = true;
    bool depthTest = true;
    bool depthWrite = true;
    bool blending = false;
};

class IPipeline
{
public:
    virtual ~IPipeline() = default;
    virtual EPipelineType GetPipelineType() const = 0;

protected:
    EPipelineType m_ePipelineType = EPipelineType::PIPELINE_TYPE_STATIC_MESH;
};
