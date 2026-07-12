#include "Device/PipelinesManager.h"
#include "API/RenderDevice.h"
#include "Device/VulkanRenderDevice.h"
#include "VulkanModel/StaticMeshData.h"
#include "Logging/LogManager.h"

bool CPipelinesManager::Initialize()
{
    // Initialize Default Pipelines
    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    // Default Static Mesh Pipeline
    const std::string& stVertexShaderName = Anubis::ResolveShaderPath(renderDev.GetAPI(), "static_mesh_shader", EShaderStage::SHADER_TYPE_VERTEX);
    const std::string& stFragmentShaderName = Anubis::ResolveShaderPath(renderDev.GetAPI(), "static_mesh_shader", EShaderStage::SHADER_TYPE_FRAGMENT);
    SPipelineDesc pipeLineDesc{};
    pipeLineDesc.pipelineType = EPipelineType::PIPELINE_TYPE_STATIC_MESH;
    pipeLineDesc.shader.m_stName = "StaticMeshShader";
    pipeLineDesc.shader.m_vStages = {
        { EShaderStage::SHADER_TYPE_VERTEX, stVertexShaderName }, // SPV
        { EShaderStage::SHADER_TYPE_FRAGMENT, stFragmentShaderName } // SPV
    };

    // No materials involved — layout comes from the shader contract itself
    if (renderDev.GetAPI() == EGraphicsAPI::API_VULKAN)
    {
        auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
        const SVulkanContext& context = vkRenderDevice.GetContext();

        pipeLineDesc.hasVertexInput = true;
        pipeLineDesc.vertexBindingDescription = SStaticMeshVertex::GetBindingDescription();
        pipeLineDesc.vertexAttributeDescriptions = SStaticMeshVertex::GetAttributeDescriptions();

        VkDescriptorSetLayout layout = CStaticMeshShaderLayout::Get(context);
        if (layout == VK_NULL_HANDLE)
        {
            syserr("Failed to create static mesh descriptor set layout");
            return false;
        }

        pipeLineDesc.descriptorSetLayouts.push_back(layout);
    }
    else if (renderDev.GetAPI() == EGraphicsAPI::API_OPENGL)
    {

    }

    pipeLineDesc.topology = EPrimitiveTopology::TOPOLOGY_TRIANGLES_LIST;
    pipeLineDesc.cullMode = ECullMode::CULL_MODE_BACK;
    pipeLineDesc.frontFace = EFrontFace::FRONT_FACE_COUNTER_CLOCKWISE;
    pipeLineDesc.depthTest = true;

    IPipeline* pStaticcMeshPipeline = GetOrCreatePipeline(pipeLineDesc);
    if (pStaticcMeshPipeline == nullptr)
    {
        syserr("Failed to create Default Pipeline Static Mesh Pipeline");
        return (false);
    }

    return (true);
}

IPipeline* CPipelinesManager::GetOrCreatePipeline(const SPipelineDesc& desc)
{
    auto key = desc.pipelineType;

    auto it = m_mapPipelines.find(desc.pipelineType);
    if (it != m_mapPipelines.end())
    {
        return it->second;
    }

    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();
    IPipeline* pPipeline = renderDev.CreatePipeline(desc);

    if (!pPipeline)
    {
        syserr("CPipelineManager: failed to create pipeline");
        return nullptr;
    }

    m_mapPipelines[key] = pPipeline;
    return pPipeline;
}

IPipeline* CPipelinesManager::GetPipeline(const EPipelineType& type)
{
    auto it = m_mapPipelines.find(type);
    if (it != m_mapPipelines.end())
    {
        return it->second;
    }

    return nullptr;
}

void CPipelinesManager::Clear()
{
    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    for (auto& [key, pipeline] : m_mapPipelines)
    {
        if (pipeline)
        {
            renderDev.DestroyPipeline(pipeline); // add this to CIRenderDevice if missing
            AnubisSafeDelete(pipeline);
        }
    }

    if (renderDev.GetAPI() == EGraphicsAPI::API_VULKAN)
    {
        auto& vkRenderDevice = static_cast<CVulkanRenderDevice&>(renderDev);
        CStaticMeshShaderLayout::Destroy(vkRenderDevice.GetContext());
    }
    m_mapPipelines.clear();
}
