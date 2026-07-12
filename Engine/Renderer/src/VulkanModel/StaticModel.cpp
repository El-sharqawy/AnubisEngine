#include "VulkanModel/StaticModel.h"
#include "Vulkan/VulkanBuffer.h"
#include "Vulkan/VulkanUtils.h"
#include "Vulkan/VulkanDevice.h"
#include "Vulkan/VulkanPipeline.h"
#include "Device/VulkanRenderDevice.h"
#include "Device/OpenGLRenderDevice.h"
#include "Textures/TexturesManager.h"
#include <stb_image/stb_image.h>
#include "EngineMathMatrix.h"
#include "EngineMath.h"
#include "EngineMathQuaternion.h"
#include <memory>
#include "Device/PipelinesManager.h"
#include "OpenGL/OpenGLTexture2D.h"

bool CStaticModel::ImportModel(const std::filesystem::path& filePath, const std::vector<IBuffer*>& vpUniformBuffer, const SModelImportOptions& options)
{
    if (!ReadScene(filePath, options))
    {
        return false;
    }

    BuildMeshes();
    BuildMaterials();

    BuildMergedGeometry();

    if (!InitializeMaterialBindings(vpUniformBuffer))
    {
        return (false);
    }

    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    m_matModel = EngineMath::Rotate(m_matModel, 270.0f, Vector3D(1.0f, 0.0f, 0.0f));
    m_matModel = EngineMath::Scale(m_matModel, Vector3D(0.01f, 0.01f, 0.01f));

    m_Importer.FreeScene();
    m_pScene = nullptr;

    return (true);
}

bool CStaticModel::ReadScene(const std::filesystem::path& filePath, const SModelImportOptions& options)
{
    if (filePath.empty())
    {
        syserr("File path is empty");
        return (false);
    }

    if (!std::filesystem::exists(filePath))
    {
        syserr("File not found: '%s'", filePath.string().c_str());
        return (false);
    }

    // Build post-processing flags
    uint32_t uiFlags = ASSIMP_LOAD_FLAGS;

    if (options.m_bFlipUVs)
    {
        uiFlags |= aiProcess_FlipUVs;
    }

    // Read the scene from disk
    m_pScene = m_Importer.ReadFile(filePath.string(), uiFlags);

    if (!m_pScene)
    {
        syserr("Assimp error parsing '%s': '%s'", filePath.string().c_str(), m_Importer.GetErrorString());
        return (false);
    }

    if (m_pScene->mFlags & AI_SCENE_FLAGS_INCOMPLETE)
    {
        syserr("Scene is incomplete: '%s'", filePath.string().c_str());
        return false;
    }

    if (!m_pScene->mRootNode)
    {
        syserr("Scene has no root node: '%s'", filePath.string().c_str());
        return false;
    }

    // Store paths for later use in material/texture loading
    m_modelPath = filePath;
    m_modelDirectory = filePath.parent_path();

    // Store global inverse transform for skinning/animation
    aiMatrix4x4 rootTransform = m_pScene->mRootNode->mTransformation;
    aiMatrix4x4 inverseTransform = rootTransform;
    inverseTransform.Inverse();

    // Convert to your engine matrix type
    // m_matGlobalInverseTransform = Matrix4(inverseTransform);

#if defined(ENABLE_MESH_LOGS)
    syslog("Loaded '{}' meshes: {}, materials: {}, textures: {}",
        filePath.string().c_str(),
        m_pScene->mNumMeshes,
        m_pScene->mNumMaterials,
        m_pScene->mNumTextures);
#endif

    return (true);
}

void CStaticModel::BuildMeshes()
{
    if (!m_pScene || m_pScene->mNumMeshes == 0)
    {
        syserr("No meshes in scene '{}'", m_modelPath.string().c_str());
        return;
    }

    m_vMeshes.reserve(m_pScene->mNumMeshes);
    m_vMeshes.resize(m_pScene->mNumMeshes);

#if defined(ENABLE_MESH_LOGS)
    syslog("Building {} meshes", m_pScene->mNumMeshes);
#endif

    uint32_t uiTotalVertices = 0;
    uint32_t uiTotalIndices = 0;

    for (size_t i = 0; i < m_pScene->mNumMeshes; i++)
    {
        aiMesh* pAiMesh = m_pScene->mMeshes[i];
        uint32_t uiMaterialIndex = pAiMesh->mMaterialIndex;
        if (!GetMeshes()[i].LoadMesh(pAiMesh, uiMaterialIndex))
        {
            syserr("Failed to load submesh {} in '{}'", i, m_modelPath.string().c_str());
            continue;
        }

        uiTotalVertices += m_vMeshes[i].GetVertexCount();
        uiTotalIndices += m_vMeshes[i].GetIndexCount();
    }

#if defined(ENABLE_MESH_LOGS)
    syslog("Total vertices: {}, indices: {}", uiTotalVertices, uiTotalIndices);
#endif
}

void CStaticModel::BuildMaterials()
{
    if (!m_pScene || m_pScene->mNumMaterials == 0)
    {
        syserr("No materials in scene '{}'", m_modelPath.string().c_str());
        return;
    }

    GetMaterials().reserve(m_pScene->mNumMaterials);

#if defined(ENABLE_MESH_LOGS)
    syslog("Building {} materials", m_pScene->mNumMaterials);
#endif

    for (uint32_t i = 0; i < m_pScene->mNumMaterials; i++)
    {
        const aiMaterial* pAiMat = m_pScene->mMaterials[i];
        IMaterial* material = ProcessMaterial(pAiMat, m_pScene);

#if defined(ENABLE_MESH_LOGS)
        syslog("  [{}] name: '{}' workflow: {}",
            i,
            material->GetMaterialName(),
            material->GetMaterialWorkflow() == EMaterialWorkflow::MATERIAL_WORKFLOW_PBR ? "PBR" : "Legacy");
        syslog("    diffuseMap: {}  albedoMap: {}  metallic: {:.2f} roughness: {:.2f}",
            material->GetMaterialDiffuseMap() ? "yes" : "no",
            material->GetMaterialPBR().m_pAlbedoMap ? "yes" : "no",
            material->GetMaterialPBR().m_fMetallic,
            material->GetMaterialPBR().m_fRoughness);
#endif

        GetMaterials().push_back(std::move(material));
    }
}

IMaterial* CStaticModel::ProcessMaterial(const aiMaterial* mat, const aiScene* scene)
{
    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    IMaterial* material = renderDev.CreateMaterial();

    if (!mat || !scene)
    {
        syserr("Null material or scene pointer");
        return material;
    }

    if (mat)
    {
        aiString name;
        if (mat->Get(AI_MATKEY_NAME, name) == AI_SUCCESS)
        {
            material->SetMaterialName(name.C_Str());
        }
    }

    // Legacy colors
    aiColor4D color;
    if (aiGetMaterialColor(mat, AI_MATKEY_COLOR_AMBIENT, &color) == AI_SUCCESS)
    {
        material->SetMaterialAmbientColor(SVector4Df(color.r, color.g, color.b, color.a));
    }
    if (aiGetMaterialColor(mat, AI_MATKEY_COLOR_DIFFUSE, &color) == AI_SUCCESS)
    {
        material->SetMaterialDiffuseColor(SVector4Df(color.r, color.g, color.b, color.a));
    }
    if (aiGetMaterialColor(mat, AI_MATKEY_COLOR_SPECULAR, &color) == AI_SUCCESS)
    {
        material->SetMaterialSpecularColor(SVector4Df(color.r, color.g, color.b, color.a));
    }

    // Transparency / alpha
    float opacity = 1.0f;
    if (mat->Get(AI_MATKEY_OPACITY, opacity) == AI_SUCCESS)
    {
        material->SetMaterialAlpha(opacity);
        material->SetMaterialTransparency(opacity);
    }

    // Legacy textures
    ITexture2D* pDiffuseMap = LoadMaterialTexture("diffuseMap", mat, aiTextureType_DIFFUSE, scene);
    if (pDiffuseMap)
    {
        material->SetMaterialDiffuseMap(pDiffuseMap);
    }
    ITexture2D* pSpecularMap = LoadMaterialTexture("specularMap", mat, aiTextureType_SPECULAR, scene);
    if (pSpecularMap)
    {
        material->SetMaterialSpecularMap(pSpecularMap);
    }

    // PBR factors
    aiColor4D baseColor;
    if (mat->Get(AI_MATKEY_BASE_COLOR, baseColor) == AI_SUCCESS)
    {
        material->SetMaterialPBRBaseColor(SVector3Df(baseColor.r, baseColor.g, baseColor.b));
    }

    float roughness = 1.0f;
    if (mat->Get(AI_MATKEY_ROUGHNESS_FACTOR, roughness) == AI_SUCCESS)
    {
        material->SetMaterialPBRRoughness(roughness);
    }

    float metallic = 0.0f;
    if (mat->Get(AI_MATKEY_METALLIC_FACTOR, metallic) == AI_SUCCESS)
    {
        material->SetMaterialPBRRoughness(roughness);
    }

    // PBR textures
    ITexture2D* pAlbedoMap = LoadMaterialTexture("albedoMap", mat, aiTextureType_BASE_COLOR, scene);
    if (pAlbedoMap)
    {
        material->SetMaterialPBRAlbedoMap(pAlbedoMap);
    }
    ITexture2D* pNormalMap = LoadMaterialTexture("normalMap", mat, aiTextureType_NORMALS, scene);
    if (pNormalMap)
    {
        material->SetMaterialPBRNormalMap(pNormalMap);
    }
    ITexture2D* pMetallicMap = LoadMaterialTexture("metallicMap", mat, aiTextureType_METALNESS, scene);
    if (pMetallicMap)
    {
        material->SetMaterialPBRMetallicMap(pMetallicMap);
    }
    ITexture2D* pRoughnessMap = LoadMaterialTexture("roughnessMap", mat, aiTextureType_DIFFUSE_ROUGHNESS, scene);
    if (pRoughnessMap)
    {
        material->SetMaterialPBRRoughnessMap(pRoughnessMap);
    }

    // Fallbacks
    if (material->GetMaterialPBR().m_v3BaseColor.x == 1.0f &&
        material->GetMaterialPBR().m_v3BaseColor.y == 1.0f &&
        material->GetMaterialPBR().m_v3BaseColor.z == 1.0f)
    {
        material->SetMaterialPBRBaseColor(Vector3D(material->GetMaterialDiffuseColor()));
    }

    // Decide workflow
    const bool hasRealPBRTexture =
        (mat->GetTextureCount(aiTextureType_BASE_COLOR) > 0) ||
        (mat->GetTextureCount(aiTextureType_NORMALS) > 0) ||
        (mat->GetTextureCount(aiTextureType_METALNESS) > 0) ||
        (mat->GetTextureCount(aiTextureType_DIFFUSE_ROUGHNESS) > 0);

    const bool hasPBRFactors =
        (material->GetMaterialPBRMetallic() != 0.0f) ||
        (material->GetMaterialPBRRoughness() != 1.0f);

    if (hasRealPBRTexture || hasPBRFactors)
    {
        material->SetMaterialWorkflow(EMaterialWorkflow::MATERIAL_WORKFLOW_PBR);
    }
    else
    {
        material->SetMaterialWorkflow(EMaterialWorkflow::MATERIAL_WORKFLOW_LEGACY);
    }

    return material;
}

ITexture2D* CStaticModel::LoadMaterialTexture(const std::string& stName, const aiMaterial* mat, aiTextureType type, const aiScene* scene)
{
    if (!mat || !scene)
    {
        return nullptr;
    }

    if (mat->GetTextureCount(type) == 0)
    {
        return nullptr;
    }

    aiString texPath;
    if (mat->GetTexture(type, 0, &texPath) != AI_SUCCESS)
    {
        return nullptr;
    }

    const std::string rawTexturePath = texPath.C_Str();
    if (rawTexturePath.empty())
    {
        return nullptr;
    }

    std::string cacheKey;
    ITexture2D* pTexture = nullptr;

    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    // Embedded texture
    // Assimp can return names like "*0", or a path that
    // scene->GetEmbeddedTexture can resolve.
    if (const aiTexture* embeddedTex = scene->GetEmbeddedTexture(rawTexturePath.c_str()))
    {
        if (embeddedTex->mHeight == 0)
        {
            // Compressed texture data (png/jpg/etc.)
            const unsigned char* data = reinterpret_cast<const unsigned char*>(embeddedTex->pcData);
            const size_t dataSize = static_cast<size_t>(embeddedTex->mWidth);
            stbi_set_flip_vertically_on_load(false);

            int width = 0;
            int height = 0;
            int originalChannels = 0;

            uint8_t* pixels = stbi_load_from_memory(reinterpret_cast<const stbi_uc*>(data), static_cast<int>(dataSize), &width, &height, &originalChannels, STBI_rgb_alpha);
            if (!pixels)
            {
                syslog("Failed to Load Texture From Memory Reason: '%s'", stbi_failure_reason());
                return (nullptr);
            }
            STextureDesc textureDesc{};
            textureDesc.m_fsFilePath.clear();
            textureDesc.m_iWidth = width;
            textureDesc.m_iHeight = height;
            textureDesc.m_iChannels = 4;
            textureDesc.m_stName = stName;

            pTexture = renderDev.CreateTexture2D(textureDesc, pixels);
            if (!pTexture)
            {
                syserr("Failed to Load Embedded Texture");
                return (nullptr);
            }

            stbi_image_free(pixels);
        }
        else
        {
            // Uncompressed ARGB8888 texel data
            const unsigned char* data = reinterpret_cast<const unsigned char*>(embeddedTex->pcData);
            const int32_t width = static_cast<int>(embeddedTex->mWidth);
            const int32_t height = static_cast<int>(embeddedTex->mHeight);

            const size_t imageSize = static_cast<size_t>(width) * static_cast<size_t>(height) * 4;
            std::vector<uint8_t> vPixels;
            vPixels.assign(data, data + imageSize);

            STextureDesc textureDesc{};
            textureDesc.m_fsFilePath.clear();
            textureDesc.m_iWidth = width;
            textureDesc.m_iHeight = height;
            textureDesc.m_iChannels = 4;
            textureDesc.m_stName = stName;

            pTexture = renderDev.CreateTexture2D(textureDesc, vPixels.data());
            if (!pTexture)
            {
                syserr("Failed to Load Embedded Texture");
                return (nullptr);
            }
        }
    }
    else
    {
        // --------------------------------------------------
        // External texture file
        // --------------------------------------------------
        std::filesystem::path baseDir = m_modelPath.parent_path();
        std::filesystem::path fullPath = (baseDir / rawTexturePath).lexically_normal();

        // Example API - adapt to your texture class
        STextureDesc textureDesc{};
        textureDesc.m_fsFilePath = fullPath;
        textureDesc.m_stName = stName;

        pTexture = renderDev.CreateTexture2D(textureDesc);
        if (!pTexture)
        {
            syserr("Failed to Load Texture From {}", fullPath.string().c_str());
            return (nullptr);
        }
    }

    return pTexture;
}

void CStaticModel::Clear()
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
	m_vBatches.clear();

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

void CStaticModel::BuildMergedGeometry()
{
    m_vMergedVertices.clear();
    m_vMergedIndices.clear();
    m_vBatches.clear();

    size_t totalVertexCount = 0;
    size_t totalIndexCount = 0;

    for (const auto& mesh : m_vMeshes)
    {
        totalVertexCount += mesh.GetVertices().size();
        totalIndexCount += mesh.GetIndices().size();
    }

    m_vMergedVertices.reserve(totalVertexCount);
    m_vMergedIndices.reserve(totalIndexCount);
    m_vBatches.reserve(m_vMeshes.size());

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

        SModelBatch batch{};
        batch.materialIndex = mesh.GetMaterialIdx();
        batch.firstIndex = currentFirstIndex;
        batch.indexCount = static_cast<uint32_t>(indices.size());
        batch.baseVertex = 0; // indices are already rebased

        m_vBatches.push_back(batch);

        currentBaseVertex += static_cast<uint32_t>(vertices.size());
        currentFirstIndex += static_cast<uint32_t>(indices.size());
    }
}

void CStaticModel::UploadToGPU()
{
    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    SBufferDesc vertexBufferDesc{};
    vertexBufferDesc.m_stName = "Model Vertex Buffer";
    vertexBufferDesc.m_eType = EBufferType::BUFFER_TYPE_VERTEX;
    vertexBufferDesc.m_uiSize = m_vMergedVertices.size() * sizeof(SStaticMeshVertex);
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
            { m_pVertexBuffer, 0, 0, sizeof(SStaticMeshVertex), 0 }
        };
        // Attribute 0 -> position.xyz
        // Attribute 1 -> color.rgba
        bufVAODesc.m_vAttribs =
        {
            // position
            { 0, 0, 3, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SStaticMeshVertex, SStaticMeshVertex::position),       0 },
            // normal
            { 1, 0, 3, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SStaticMeshVertex, SStaticMeshVertex::normal),   0 },
            // texcoord
            { 2, 0, 2, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SStaticMeshVertex, SStaticMeshVertex::texCoord),   0 },
            // tangent
            { 3, 0, 3, GL_FLOAT, EVertexAttribClass::Float, GL_FALSE, offsetof(SStaticMeshVertex, SStaticMeshVertex::tangent),   0 },
        };

        m_pVertexArray = glRenderDevice.CreateVertexArray(bufVAODesc);
        if (m_pVertexArray == nullptr)
        {
            syserr("Failed to Create Vertex Array for Static Model");
        }
    }
}

void CStaticModel::RenderModel(uint32_t currentFrame, ICommandList* pCmd)
{
    if (!m_pVertexBuffer || !m_pIndexBuffer || !pCmd)
    {
        syserr("RenderModel: missing buffer or command list");
        return;
    }


    if (!m_pVertexBuffer->IsValid() || !m_pIndexBuffer->IsValid())
    {
        syserr("RenderModel: Vulkan buffer handle is null");
        return;
    }

    auto& pipesManager = CPipelinesManager::Instance();

    pCmd->BindPipeline(pipesManager.GetPipeline(EPipelineType::PIPELINE_TYPE_STATIC_MESH));
    glBindVertexArray(m_pVertexArray->GetID());

    pCmd->BindVertexBuffer(m_pVertexBuffer);
    pCmd->BindIndexBuffer(m_pIndexBuffer, EIndexType::INDEX_TYPE_UINT32);

    SPushConstantModel modelData{};
    modelData.matModel = GetModelMatrix();
    pCmd->PushConstants(&modelData, sizeof(modelData));

    for (const auto& batch : m_vBatches)
    {
        IMaterial* mat = GetMaterial(batch.materialIndex);
        pCmd->BindMaterial(mat, currentFrame);
        pCmd->DrawIndexed(batch.indexCount, 1, batch.firstIndex);
    }
}

std::vector<SRenderItem> CStaticModel::BuildRenderItems()
{
    auto* pipeline = CPipelinesManager::Instance().GetPipeline(EPipelineType::PIPELINE_TYPE_STATIC_MESH);
    std::vector<SRenderItem> renderItems{};

    for (const auto& batch : m_vBatches)
    {
        SRenderItem item{};
        item.pPipeline = pipeline;
        item.pMaterial = GetMaterial(batch.materialIndex);
        item.pVertexBuffer = m_pVertexBuffer;
        item.pIndexBuffer = m_pIndexBuffer;
        item.pVertexArray = m_pVertexArray;
        item.indexCount = batch.indexCount;
        item.firstIndex = batch.firstIndex;
        item.modelMatrix = GetModelMatrix();
        item.sortKey = 0;
        renderItems.push_back(item);
    }

    return (renderItems);
}

Matrix4 CStaticModel::GetModelMatrix()
{
    if (m_bTransformDirty)
    {
        m_matModel = Matrix4(1.0f);
        m_matModel = EngineMath::Translate(m_matModel, m_v3Position);
        SQuaternion qModelFix = EngineMath::FromXRotation(EngineMath::ToRadians(270.0f), false); // Rotate 180 degrees on X Axis
        SQuaternion qFinalRot = EngineMath::Multiply(m_qRotation, qModelFix);
        m_matModel = m_matModel * EngineMath::ToMatrix4(qFinalRot);
        m_matModel = EngineMath::Scale(m_matModel, m_v3Scale);
        m_bTransformDirty = false;
    }

    return (m_matModel);
}

void CStaticModel::SetPosition(const Vector3D& v3Pos)
{
    m_v3Position = v3Pos;
    m_bTransformDirty = true;
}

void CStaticModel::SetScale(const Vector3D& v3Scale)
{
    m_v3Scale = v3Scale;
    m_bTransformDirty = true;
}

bool CStaticModel::InitializeMaterialBindings(const std::vector<IBuffer*>& vpUniformBuffer)
{
    ITexture2D* pFallbackWhite = CTexturesManager::Instance().GetFallBackWhiteTexture();
    ITexture2D* pFallbackNormal = CTexturesManager::Instance().GetFallBackNormalTexture();
    ITexture2D* pFallbackBlack = CTexturesManager::Instance().GetFallBackBlackTexture();

    for (IMaterial* material : GetMaterials())
    {
        SBindingContextDesc ctxDesc{};
        ctxDesc.m_uiFrameCount = static_cast<uint32_t>(vpUniformBuffer.size());

        ctxDesc.m_vBindings = CStaticMeshShaderLayout::GetBindings();

        ctxDesc.m_vImageResources.push_back({ 1, material->GetMaterialDiffuseMap() ? material->GetMaterialDiffuseMap() : pFallbackWhite });
        ctxDesc.m_vImageResources.push_back({ 2, material->GetMaterialSpecularMap() ? material->GetMaterialSpecularMap() : pFallbackWhite });
        ctxDesc.m_vImageResources.push_back({ 3, material->GetMaterialPBRAlbedoMap() ? material->GetMaterialPBRAlbedoMap() : pFallbackWhite });
        ctxDesc.m_vImageResources.push_back({ 4, material->GetMaterialPBRNormalMap() ? material->GetMaterialPBRNormalMap() : pFallbackNormal });
        ctxDesc.m_vImageResources.push_back({ 5, material->GetMaterialPBRMetallicMap() ? material->GetMaterialPBRMetallicMap() : pFallbackWhite });
        ctxDesc.m_vImageResources.push_back({ 6, material->GetMaterialPBRRoughnessMap() ? material->GetMaterialPBRRoughnessMap() : pFallbackWhite });

        syslog("fallback white tex ptr={} id={} valid={}",
            (void*)pFallbackWhite,
            pFallbackWhite ? static_cast<COpenGLTexture2D*>(pFallbackWhite)->GetTextureID() : 0,
            pFallbackWhite ? static_cast<COpenGLTexture2D*>(pFallbackWhite)->IsValid() : false);

        SBindingBufferResource bufferRes{};
        bufferRes.m_uiBinding = 0;
        bufferRes.m_vBuffers = vpUniformBuffer;
        ctxDesc.m_vBufferResources.push_back(bufferRes);

        if (!material->InitializeMaterial(ctxDesc))
        {
            syserr("Failed to Initialize Material {}", material->GetMaterialName());
            return (false);
        }
    }

    return (true);
}
