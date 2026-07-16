#pragma once

#include "API/Material.h"
#include "API/Texture.h"

#include "Model/StaticMeshData.h"
#include "Model/SkeletalMeshData.h"
#include <assimp/Importer.hpp>      // C++ importer interface
#include <assimp/scene.h>			// Output data structure
#include <assimp/postprocess.h>		// Post processing flags

#include <unordered_map>
#include "TypeMatrix4.h"

// #define ENABLE_MESH_LOGS

#define ASSIMP_LOAD_FLAGS (aiProcess_JoinIdenticalVertices |    \
                           aiProcess_Triangulate |              \
                           aiProcess_GenSmoothNormals |         \
                           aiProcess_LimitBoneWeights |         \
                           aiProcess_SplitLargeMeshes |         \
                           aiProcess_ImproveCacheLocality |     \
                           aiProcess_RemoveRedundantMaterials | \
                           aiProcess_FindDegenerates |          \
                           aiProcess_FindInvalidData |          \
                           aiProcess_GenUVCoords |              \
                           aiProcess_CalcTangentSpace)

class CStaticModel;
class CSkeletalModel;
class CModelAssetBase;
class CStaticActorAsset;
class CActor;
class CActorAssetBase;
class CSkeleton;

struct SModelImportOptions
{
    bool m_bFlipUVs = false;
};

enum class EModelImportType
{
    IMPORT_TYPE_AUTO,
    IMPORT_TYPE_STATIC,
    IMPORT_TYPE_SKELETAL,
};

class CAssimpModelImporter
{
public:
    // std::shared_ptr<CActor> ImportActor(const std::filesystem::path& filePath, const SModelImportOptions& options = {});
    // std::shared_ptr<CActorAsset> ImportActorAsset(const std::filesystem::path& filePath, const SModelImportOptions& options = {});
    // std::shared_ptr<CModel> ImportModel(const std::filesystem::path& filePath, const SModelImportOptions& options = {});

    std::shared_ptr<CActor> ImportActor(const std::filesystem::path& filePath, const SModelImportOptions& options = {}, EModelImportType importType = EModelImportType::IMPORT_TYPE_AUTO);

    std::shared_ptr<CActorAssetBase> ImportActorAsset(const std::filesystem::path& filePath, const SModelImportOptions& options = {}, EModelImportType importType = EModelImportType::IMPORT_TYPE_AUTO);
    std::shared_ptr<CModelAssetBase> ImportModel(const std::filesystem::path& filePath, const SModelImportOptions& options = {}, EModelImportType importType = EModelImportType::IMPORT_TYPE_AUTO);
    std::shared_ptr<CSkeletalModel> ImportSkeletalModel(const std::filesystem::path& filePath, const SModelImportOptions& options = {});
    std::shared_ptr<CStaticModel> ImportStaticModel(const std::filesystem::path& filePath, const SModelImportOptions& options = {});

    std::shared_ptr<CStaticModel> BuildStaticModelAsset(const std::filesystem::path& filePath);
    std::shared_ptr<CSkeletalModel> BuildSkeletalModelAsset(const std::filesystem::path& filePath);

    void ClearScene();

private:
    bool ReadScene(const std::filesystem::path& filePath, const SModelImportOptions& options);
    bool BuildStaticMeshes(std::shared_ptr<CStaticModel> pModel);

    bool BuildMaterials(std::shared_ptr<CModelAssetBase> pModel);
    IMaterial* ProcessMaterial(const aiMaterial* mat, const aiScene* scene);
    ITexture2D* LoadMaterialTexture(const std::string& stName, const aiMaterial* mat, aiTextureType type, const aiScene* scene);

    bool BuildSkeleton(std::shared_ptr<CSkeleton> pSkeleton);
    bool BuildSkeletalMeshes(std::shared_ptr<CSkeletalModel> pModel, std::shared_ptr<CSkeleton> pSkeleton);

    bool HasBones() const;

private:
    Assimp::Importer m_Importer;
    const aiScene* m_pScene = nullptr; // owns m_pScene lifetime
    std::filesystem::path m_modelPath;
    std::filesystem::path m_modelDirectory;
    Matrix4 m_matGlobalInverseTransform;

    std::unordered_map<std::string, ITexture2D*> m_mLoadedTextures;
};