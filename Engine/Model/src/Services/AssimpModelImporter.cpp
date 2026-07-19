#include "Services/AssimpModelImporter.h"
#include "Model/StaticModel.h"
#include "Model/SkeletalModel.h"
#include "Model/Skeleton.h"
#include "Model/StaticActorAsset.h"
#include "Model/SkeletalActorAsset.h"
#include "Logging/LogManager.h"
#include "API/RenderDevice.h"

std::shared_ptr<CActorAssetBase> CAssimpModelImporter::ImportActorAsset(const std::filesystem::path& filePath, const SModelImportOptions& options, EModelImportType importType)
{
    std::shared_ptr<CModelAssetBase> pModel = ImportModel(filePath, options, importType);

    if (!pModel)
    {
        return nullptr;
    }

    if (pModel->GetModelType() == EModelAssetType::MODEL_TYPE_SKELETAL)
    {
        auto pActorAsset = std::make_shared<CSkeletalActorAsset>();
        pActorAsset->SetName(filePath.stem().string());
        pActorAsset->SetModelPath(filePath.string());
        pActorAsset->SetModelAsset(pModel);
        pActorAsset->SetSkeletalModel(std::static_pointer_cast<CSkeletalModel>(pModel));
        return pActorAsset;
    }
    else
    {
        auto pActorAsset = std::make_shared<CStaticActorAsset>();
        pActorAsset->SetName(filePath.stem().string());
        pActorAsset->SetModelPath(filePath.string());
        pActorAsset->SetModelAsset(pModel);
        pActorAsset->SetStaticModel(std::static_pointer_cast<CStaticModel>(pModel));
        return pActorAsset;
    }
}

std::shared_ptr<CModelAssetBase> CAssimpModelImporter::ImportModel(const std::filesystem::path& filePath, const SModelImportOptions& options, EModelImportType importType)
{
    if (!ReadScene(filePath, options))
    {
        return nullptr;
    }

    std::shared_ptr<CModelAssetBase> model;

    switch (importType)
    {
    case EModelImportType::IMPORT_TYPE_AUTO:
        if (HasBones())
        {
            model = BuildSkeletalModelAsset(filePath);
        }
        else
        {
            model = BuildStaticModelAsset(filePath);
        }
        break;

    case EModelImportType::IMPORT_TYPE_STATIC:
        model = BuildStaticModelAsset(filePath);
        break;

    case EModelImportType::IMPORT_TYPE_SKELETAL:
        if (!HasBones())
        {
            ClearScene();
            return nullptr;
        }

        model = BuildSkeletalModelAsset(filePath);
        break;

    default:
        if (HasBones())
        {
            model = BuildSkeletalModelAsset(filePath);
        }
        else
        {
            model = BuildStaticModelAsset(filePath);
        }
        break;
    }

    return (model);
}

std::shared_ptr<CStaticModel> CAssimpModelImporter::ImportStaticModel(const std::filesystem::path& filePath, const SModelImportOptions& options)
{
    if (!ReadScene(filePath, options))
    {
        return nullptr;
    }

    std::shared_ptr<CStaticModel> model = std::make_shared<CStaticModel>();

    BuildStaticMeshes(model);
    BuildMaterials(model);

    model->BuildMergedGeometry();

    ClearScene();

    return model;
}

std::shared_ptr<CStaticModel> CAssimpModelImporter::BuildStaticModelAsset(const std::filesystem::path& filePath)
{
    auto model = std::make_shared<CStaticModel>();
    model->SetMeshFilePath(filePath.string());
    model->SetMeshName(filePath.stem().string());

    if (!BuildStaticMeshes(model))
    {
        return nullptr;
    }

    if (!BuildMaterials(model))
    {
        return nullptr;
    }

    model->BuildMergedGeometry();
    model->SetModelType(EModelAssetType::MODEL_TYPE_SKELETAL);

    return model;

}

std::shared_ptr<CSkeletalModel> CAssimpModelImporter::BuildSkeletalModelAsset(const std::filesystem::path& filePath)
{
    auto model = std::make_shared<CSkeletalModel>();
    model->SetMeshFilePath(filePath.string());
    model->SetMeshName(filePath.stem().string());

    auto skeleton = std::make_shared<CSkeleton>();
    if (!BuildSkeleton(skeleton))
    {
        return nullptr;
    }
    skeleton->SetGlobalInverseTransformMatrix(m_matGlobalInverseTransform);

    if (!BuildSkeletalMeshes(model, skeleton))
    {
        return nullptr;
    }

    if (!BuildMaterials(model))
    {
        return nullptr;
    }

    model->SetSkeleton(skeleton);
    model->BuildMergedGeometry();
    model->SetModelType(EModelAssetType::MODEL_TYPE_SKELETAL);

    return model;

}

void CAssimpModelImporter::ClearScene()
{
    m_matGlobalInverseTransform = {};
    m_Importer.FreeScene();
    m_pScene = nullptr;
}

bool CAssimpModelImporter::ReadScene(const std::filesystem::path& filePath, const SModelImportOptions& options)
{
    if (filePath.empty())
    {
        syserr("File path is empty");
        return (false);
    }

    if (!std::filesystem::exists(filePath))
    {
        syserr("File not found: '{}'", filePath.string());
        return (false);
    }

    // Build post-processing flags
    unsigned int uiFlags = ASSIMP_LOAD_FLAGS;

    if (options.m_bFlipUVs)
    {
        uiFlags |= aiProcess_FlipUVs;
    }

    // Read the scene from disk
    m_pScene = m_Importer.ReadFile(filePath.string(), uiFlags);


    if (!m_pScene)
    {
        syserr("Assimp error parsing '{}': '{}'", filePath.string().c_str(), m_Importer.GetErrorString());
        return (false);
    }

    if (m_pScene->mFlags & AI_SCENE_FLAGS_INCOMPLETE)
    {
        syserr("Scene is incomplete: '{}'", filePath.string());
        return false;
    }

    if (!m_pScene->mRootNode)
    {
        syserr("Scene has no root node: '{}'", filePath.string());
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
    m_matGlobalInverseTransform = Anubis::AssimpToMatrix4(inverseTransform);

#if defined(ENABLE_MESH_LOGS)
    syslog("Loaded '{}' meshes: {}, materials: {}, textures: {}",
        filePath.string(),
        m_pScene->mNumMeshes,
        m_pScene->mNumMaterials,
        m_pScene->mNumTextures);
#endif

    return (true);
}

bool CAssimpModelImporter::BuildStaticMeshes(std::shared_ptr<CStaticModel> pModel)
{
    if (!m_pScene || m_pScene->mNumMeshes == 0)
    {
        syserr("No meshes in scene '{}'", m_modelPath.string());
        return (false);
    }

    auto& meshes = pModel->GetMeshes();
    meshes.clear();
    meshes.resize(m_pScene->mNumMeshes);

#if defined(ENABLE_MESH_LOGS)
    syslog("Building {} meshes", m_pScene->mNumMeshes);
#endif

    GLuint uiTotalVertices = 0;
    GLuint uiTotalIndices = 0;

    for (size_t i = 0; i < m_pScene->mNumMeshes; i++)
    {
        aiMesh* pAiMesh = m_pScene->mMeshes[i];
        uint32_t uiMaterialIndex = pAiMesh->mMaterialIndex;

        if (!meshes[i].LoadMesh(pAiMesh, uiMaterialIndex))
        {
            syserr("Failed to load submesh {} in '{}'", i, m_modelPath.string());
            continue;
        }

        uiTotalVertices += meshes[i].GetVertexCount();
        uiTotalIndices += meshes[i].GetIndexCount();
    }

#if defined(ENABLE_MESH_LOGS)
    syslog("Total vertices: {}, indices: {}", uiTotalVertices, uiTotalIndices);
#endif

    return (true);
}

bool CAssimpModelImporter::BuildMaterials(std::shared_ptr<CModelAssetBase> pModel)
{
    if (!m_pScene || m_pScene->mNumMaterials == 0)
    {
        syserr("No materials in scene '{}'", m_modelPath.string().c_str());
        return (false);
    }

    auto& materials = pModel->GetMaterials();
    materials.clear();
    materials.reserve(m_pScene->mNumMaterials);

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

        materials.push_back(std::move(material));
    }

    return (true);
}

IMaterial* CAssimpModelImporter::ProcessMaterial(const aiMaterial* mat, const aiScene* scene)
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

ITexture2D* CAssimpModelImporter::LoadMaterialTexture(const std::string& stName, const aiMaterial* mat, aiTextureType type, const aiScene* scene)
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

    std::filesystem::path importedPath = std::filesystem::path(texPath.C_Str()).lexically_normal();

    std::filesystem::path relativeTail;
    bool foundCharacters = false;

    for (const auto& part : importedPath)
    {
        if (foundCharacters)
        {
            relativeTail /= part;
        }
        else if (part == "Characters")
        {
            foundCharacters = true;
        }
    }

    std::filesystem::path finalPath;
    if (!relativeTail.empty())
    {
        finalPath = std::filesystem::path("Assets\\Characters") / relativeTail;
    }
    else
    {
        finalPath = std::filesystem::path("Assets\\Characters") / importedPath.filename();
    }

    if (!std::filesystem::exists(finalPath))
    {
        syserr("Texture file does not exist: '{}'", finalPath.string());
        return (nullptr);
    }

    std::string cacheKey;
    ITexture2D* pTexture = nullptr;

    auto& renderDev = CServiceLocator::Get<CIRenderDevice>();

    // Embedded texture
    // Assimp can return names like "*0", or a path that
    // scene->GetEmbeddedTexture can resolve.
    if (const aiTexture* embeddedTex = scene->GetEmbeddedTexture(finalPath.string().c_str()))
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
        textureDesc.m_fsFilePath = finalPath;
        textureDesc.m_stName = stName;

        pTexture = renderDev.CreateTexture2D(textureDesc);
        if (!pTexture)
        {
            syserr("Failed to Load Texture From {}", finalPath.string().c_str());
            return (nullptr);
        }
    }

    return pTexture;
}

bool CAssimpModelImporter::BuildSkeleton(std::shared_ptr<CSkeleton> pSkeleton)
{
    if (!m_pScene || !m_pScene->mRootNode)
    {
        syserr("Scene or root node is null.");
        return (false);
    }

    auto& boneInfoMap = pSkeleton->GetBoneInfoMap();
    int boneCount = 0;

    for (uint32_t iMesh = 0; iMesh < m_pScene->mNumMeshes; iMesh++)
    {
        aiMesh* pMesh = m_pScene->mMeshes[iMesh];
        if (!pMesh)
        {
            continue;
        }

        for (uint32_t iBone = 0; iBone < pMesh->mNumBones; iBone++)
        {
            aiBone* pAiBone = pMesh->mBones[iBone];
            if (!pAiBone)
            {
                continue;
            }

            int boneID = -1;
            const std::string stBoneName = pAiBone->mName.C_Str();

            if (boneInfoMap.find(stBoneName) == boneInfoMap.end())
            {
                SBoneInfo newBoneInfo;
                newBoneInfo.iBoneID = boneCount;
                newBoneInfo.matOffset = Anubis::AssimpToMatrix4(pMesh->mBones[iBone]->mOffsetMatrix);
                newBoneInfo.stName = stBoneName;
                boneInfoMap[stBoneName] = newBoneInfo;
                boneID = boneCount;
                boneCount++;
            }
            else
            {
                boneID = boneInfoMap[stBoneName].iBoneID;
            }

            assert(boneID != -1);
        }
    }

    pSkeleton->SetBoneCount(boneCount);
    return (true);
}

bool CAssimpModelImporter::BuildSkeletalMeshes(std::shared_ptr<CSkeletalModel> pModel, std::shared_ptr<CSkeleton> pSkeleton)
{
    if (!m_pScene || m_pScene->mNumMeshes == 0)
    {
        syserr("No meshes in scene '{}'", m_modelPath.string());
        return (false);
    }

    auto& meshes = pModel->GetMeshes();
    meshes.clear();
    meshes.resize(m_pScene->mNumMeshes);

#if defined(ENABLE_MESH_LOGS)
    syslog("Building {} meshes", m_pScene->mNumMeshes);
#endif

    GLuint uiTotalVertices = 0;
    GLuint uiTotalIndices = 0;

    for (size_t i = 0; i < m_pScene->mNumMeshes; i++)
    {
        aiMesh* pAiMesh = m_pScene->mMeshes[i];
        uint32_t uiMaterialIndex = pAiMesh->mMaterialIndex;

        if (!meshes[i].LoadMesh(pAiMesh, uiMaterialIndex, *pSkeleton))
        {
            syserr("Failed to load submesh {} in '{}'", i, m_modelPath.string());
            continue;
        }

        uiTotalVertices += meshes[i].GetVertexCount();
        uiTotalIndices += meshes[i].GetIndexCount();
    }

#if defined(ENABLE_MESH_LOGS)
    syslog("Total vertices: {}, indices: {}", uiTotalVertices, uiTotalIndices);
#endif

    return (true);
}

bool CAssimpModelImporter::HasBones() const
{
    if (!m_pScene)
        return false;

    for (uint32_t i = 0; i < m_pScene->mNumMeshes; ++i)
    {
        if (m_pScene->mMeshes[i] && m_pScene->mMeshes[i]->mNumBones > 0)
            return true;
    }

    return false;
}

