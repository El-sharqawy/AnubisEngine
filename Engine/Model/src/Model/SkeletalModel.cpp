#include "Model/SkeletalModel.h"
#include "Device/OpenGLRenderDevice.h"
#include "Logging/LogManager.h"
#include "Device/PipelinesManager.h"
#include "Textures/TexturesManager.h"
#include "EngineMath.h"
#include "EngineMathMatrix.h"
#include "EngineMathQuaternion.h"
#include "Vulkan/VulkanDescriptorSet.h"

void CSkeletalModel::Clear()
{
    // Destroy Descriptors
    for (auto& mesh : m_vMeshes)
    {
        mesh.Unload();
    }

    for (auto& material : GetMaterials())
    {
        if (material)
        {
            material->ClearMaterial();
            AnubisSafeDelete(material);
        }
    }

    m_vMeshes.clear(); // Now it's safe to clear the vector
    m_vMaterials.clear(); // Now it's safe to clear the vector

    m_vMergedVertices.clear();
    m_vMergedIndices.clear();
    m_vNewBatches.clear();

    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    if (renderDev.GetAPI() == EGraphicsAPI::API_OPENGL)
    {
        auto& glRenderDevice = static_cast<COpenGLRenderDevice&>(renderDev);
        if (m_pVertexArray)
        {
            glRenderDevice.DestroyVertexArray(m_pVertexArray);
            AnubisSafeDelete(m_pVertexArray);
        }
    }

    if (m_pVertexBuffer)
    {
        renderDev.DestroyBuffer(m_pVertexBuffer);
        AnubisSafeDelete(m_pVertexBuffer);
    }

    if (m_pIndexBuffer)
    {
        renderDev.DestroyBuffer(m_pIndexBuffer);
        AnubisSafeDelete(m_pIndexBuffer);
    }
}

void CSkeletalModel::BuildMergedGeometry()
{
    m_vMergedVertices.clear();
    m_vMergedIndices.clear();

    size_t totalVertexCount = 0;
    size_t totalIndexCount = 0;

    for (const auto& mesh : m_vMeshes)
    {
        totalVertexCount += mesh.GetVertices().size();
        totalIndexCount += mesh.GetIndices().size();
    }

    m_vMergedVertices.reserve(totalVertexCount);
    m_vMergedIndices.reserve(totalIndexCount);
    m_vNewBatches.reserve(m_vMeshes.size());

    uint32_t currentBaseVertex = 0;
    uint32_t currentFirstIndex = 0;

    for (const auto& mesh : m_vMeshes)
    {
        const auto& vertices = mesh.GetVertices();
        const auto& indices = mesh.GetIndices();

        if (vertices.empty() || indices.empty())
            continue;

        m_vMergedVertices.insert(
            m_vMergedVertices.end(),
            vertices.begin(),
            vertices.end());

        for (uint32_t idx : indices)
        {
            m_vMergedIndices.push_back(currentBaseVertex + idx);
        }

        SMeshBatch meshBatch{};
        meshBatch.materialIndex = mesh.GetMaterialIdx();
        meshBatch.firstIndex = currentFirstIndex;
        meshBatch.indexCount = static_cast<uint32_t>(indices.size());;
        meshBatch.baseVertex = 0;  // indices are already rebased
        m_vNewBatches.push_back(meshBatch);

        currentBaseVertex += static_cast<uint32_t>(vertices.size());
        currentFirstIndex += static_cast<uint32_t>(indices.size());
    }

    UploadToGPU(); // create Vertex Buffer/Array and Index Buffer;

    // Setup GPU Part
    auto* pipeline = CPipelinesManager::Instance().GetPipeline(EPipelineType::PIPELINE_TYPE_SKELETAL_MESH);
    for (auto& newBatch : m_vNewBatches)
    {
        newBatch.pPipeline = pipeline;
        newBatch.pMaterial = GetMaterial(newBatch.materialIndex);
        newBatch.pVertexBuffer = m_pVertexBuffer;
        newBatch.pIndexBuffer = m_pIndexBuffer;
        newBatch.pVertexArray = m_pVertexArray;

        newBatch.meshId = 0; // reserved for future use
        newBatch.pipelineId = 0; // reserved for future use
        newBatch.materialId = 0; // reserved for future use
    }

    InitializeMaterialBindings();
}

void CSkeletalModel::UploadToGPU()
{
    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    SBufferDesc vertexBufferDesc{};
    vertexBufferDesc.m_stName = "Skeletal Mesh Vertex Buffer";
    vertexBufferDesc.m_eType = EBufferType::BUFFER_TYPE_VERTEX;
    vertexBufferDesc.m_uiSize = m_vMergedVertices.size() * sizeof(SSkeletalMeshVertex);
    vertexBufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
    vertexBufferDesc.cpuWrite = true;

    IBuffer* vertexBuffer = renderDev.CreateBuffer(vertexBufferDesc, m_vMergedVertices.data());
    if (!vertexBuffer)
    {
        syserr("Failed to Create Model Vertex Buffer");
        return;
    }

    m_pVertexBuffer = vertexBuffer;

    SBufferDesc indexBufferDesc{};
    indexBufferDesc.m_stName = "Model Index Buffer";
    indexBufferDesc.m_eType = EBufferType::BUFFER_TYPE_INDEX;
    indexBufferDesc.m_uiSize = m_vMergedIndices.size() * sizeof(uint32_t);
    indexBufferDesc.m_eMemoryType = EBufferMemoryType::BUFFER_MEMORY_CPU_WRITE;
    vertexBufferDesc.cpuWrite = true;

    IBuffer* indexBuffer = renderDev.CreateBuffer(indexBufferDesc, m_vMergedIndices.data());
    if (!indexBuffer)
    {
        syserr("Failed to Create Model Index Buffer");
        return;
    }
    m_pIndexBuffer = indexBuffer;

    if (renderDev.GetAPI() == EGraphicsAPI::API_OPENGL)
    {
        auto& glRenderDevice = static_cast<COpenGLRenderDevice&>(renderDev);

        SVertexArrayDesc bufVAODesc = { };
        bufVAODesc.m_stName = "ModelVAO";
        bufVAODesc.m_pIndexBuffer = m_pIndexBuffer;
        bufVAODesc.m_eIndexType = GL_UNSIGNED_INT;
        // One vertex stream: SLinesVertex { position, color }
        bufVAODesc.m_vBindings =
        {
            { m_pVertexBuffer, 0, 0, sizeof(SSkeletalMeshVertex), 0 }
        };
        // Attribute 0 -> position.xyz
        // Attribute 1 -> color.rgba
        bufVAODesc.m_vAttribs =
        {
            // position
            { 0, 0, 3, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SSkeletalMeshVertex, SSkeletalMeshVertex::position),       0 },
            // normal
            { 1, 0, 3, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SSkeletalMeshVertex, SSkeletalMeshVertex::normal),   0 },
            // texcoord
            { 2, 0, 2, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SSkeletalMeshVertex, SSkeletalMeshVertex::texCoord),   0 },
            // tangent
            { 3, 0, 4, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SSkeletalMeshVertex, SSkeletalMeshVertex::tangent),   0 },
            // bitangent
            { 4, 0, 3, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SSkeletalMeshVertex, SSkeletalMeshVertex::bitangent), 0},
            // color
            { 5, 0, 4, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SSkeletalMeshVertex, SSkeletalMeshVertex::color),     0 },
            // bone indices (as unsigned bytes, integer attribute)
            { 6, 0, 4, GL_INT, EVertexAttribClass::Integer, GL_FALSE, offsetof(SSkeletalMeshVertex, SSkeletalMeshVertex::boneIndices), 0 },
            // bone weights
            { 7, 0, 4, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SSkeletalMeshVertex, SSkeletalMeshVertex::boneWeights), 0 },
        };

        m_pVertexArray = glRenderDevice.CreateVertexArray(bufVAODesc);
        if (m_pVertexArray == nullptr)
        {
            syserr("Failed to Create Vertex Array for Static Model");
        }
    }
}

CVertexArray* CSkeletalModel::GetVertexArray()
{
    return (m_pVertexArray);
}

IBuffer* CSkeletalModel::GetVertexBuffer()
{
    return (m_pVertexBuffer);
}

IBuffer* CSkeletalModel::GetIndexBuffer()
{
    return (m_pIndexBuffer);
}

bool CSkeletalModel::InitializeMaterialBindings()
{
    ITexture2D* pFallbackWhite = CTexturesManager::Instance().GetFallBackWhiteTexture();
    ITexture2D* pFallbackNormal = CTexturesManager::Instance().GetFallBackNormalTexture();
    ITexture2D* pFallbackBlack = CTexturesManager::Instance().GetFallBackBlackTexture();

    for (IMaterial* material : GetMaterials())
    {
        SBindingContextDesc ctxDesc{};
        ctxDesc.m_uiFrameCount = MAX_FRAMES_IN_FLIGHT;

        ctxDesc.m_vBindings = CDescriptorSetLayouts::GetMaterialsBindings();

        ctxDesc.m_vImageResources.push_back({ static_cast<uint32_t>(EMaterialBindingSets::BINDING_POINT_MATERIAL_SET_SAMPLER_0), material->GetMaterialDiffuseMap() ? material->GetMaterialDiffuseMap() : pFallbackWhite });
        ctxDesc.m_vImageResources.push_back({ static_cast<uint32_t>(EMaterialBindingSets::BINDING_POINT_MATERIAL_SET_SAMPLER_1), material->GetMaterialSpecularMap() ? material->GetMaterialSpecularMap() : pFallbackWhite });
        ctxDesc.m_vImageResources.push_back({ static_cast<uint32_t>(EMaterialBindingSets::BINDING_POINT_MATERIAL_SET_SAMPLER_2), material->GetMaterialPBRAlbedoMap() ? material->GetMaterialPBRAlbedoMap() : pFallbackWhite });
        ctxDesc.m_vImageResources.push_back({ static_cast<uint32_t>(EMaterialBindingSets::BINDING_POINT_MATERIAL_SET_SAMPLER_3), material->GetMaterialPBRNormalMap() ? material->GetMaterialPBRNormalMap() : pFallbackNormal });
        ctxDesc.m_vImageResources.push_back({ static_cast<uint32_t>(EMaterialBindingSets::BINDING_POINT_MATERIAL_SET_SAMPLER_4), material->GetMaterialPBRMetallicMap() ? material->GetMaterialPBRMetallicMap() : pFallbackWhite });
        ctxDesc.m_vImageResources.push_back({ static_cast<uint32_t>(EMaterialBindingSets::BINDING_POINT_MATERIAL_SET_SAMPLER_5), material->GetMaterialPBRRoughnessMap() ? material->GetMaterialPBRRoughnessMap() : pFallbackWhite });

        if (!material->InitializeMaterial(ctxDesc))
        {
            syserr("Failed to Initialize Material {}", material->GetMaterialName());
            return (false);
        }
    }

    return (true);
}