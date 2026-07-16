#pragma once

#include "BoundingBox.h"
#include "TypeVector3.h"
#include "Quaternion.h"

#include "API/Pipeline.h"
#include "API/Material.h"
#include "API/Buffer.h"
#include "TypeMatrix4.h"

class CVertexArray;

enum ERenderItemFlags : uint32_t
{
    RENDER_ITEM_NONE = 0,
    RENDER_ITEM_SKINNED = 1 << 0,
};

/// <summary>
/// New Rendering Methods
/// </summary>
struct SMeshBatch
{
    uint32_t materialIndex = 0;
    uint32_t firstIndex = 0;
    uint32_t indexCount = 0;
    int32_t  baseVertex = 0;

    IPipeline* pPipeline = nullptr;
    IMaterial* pMaterial = nullptr;
    IBuffer* pVertexBuffer = nullptr;
    IBuffer* pIndexBuffer = nullptr;
    CVertexArray* pVertexArray = nullptr;

    uint32_t meshId = 0;
    uint32_t pipelineId = 0;
    uint32_t materialId = 0;
};

struct SRenderInstance
{
    const SMeshBatch* pBatch = nullptr;

    Matrix4 modelMatrix{ 1.0f };

    uint32_t skinPaletteIndex = UINT32_MAX;
    uint32_t flags = 0;

    float depth = 0.0f;
    uint64_t sortKey = 0;
};

/// <summary>
/// Old Rendering Methods
/// </summary>
struct SRenderItemBatch
{
    uint32_t materialIndex = 0;
    uint32_t firstIndex = 0;   // index into merged index buffer
    uint32_t indexCount = 0;
    int32_t  baseVertex = 0;   // usually 0 if indices are already rebased
};

struct SRenderItem
{
    IPipeline* pPipeline = nullptr;
    IMaterial* pMaterial = nullptr;
    IBuffer* pVertexBuffer = nullptr;
    IBuffer* pIndexBuffer = nullptr;

    CVertexArray* pVertexArray = nullptr;

    uint32_t    indexCount = 0;
    uint32_t    firstIndex = 0;

    Matrix4   modelMatrix{ 1.0f };
    std::vector<Matrix4> bonesMetrices{};
    uint32_t bUseSkinning = 0;
    uint64_t    sortKey = 0;
};

struct SModelResource
{
    uint32_t m_uiBinding;
    std::vector<IBuffer*> m_vBuffers;
};
enum EModelAssetType
{
    MODEL_TYPE_STATIC,
    MODEL_TYPE_SKELETAL,
};

class CMeshAssetBase
{
public:
    virtual ~CMeshAssetBase() = default;

    uint32_t GetBaseVertex() const { return m_uiBaseVertex; }
    uint32_t GetBaseIndex() const { return m_uiBaseIndex; }
    uint32_t GetIndexCount() const { return m_uiIndexCount; }
    uint32_t GetVertexCount() const { return m_uiVertexCount; }
    uint32_t GetMaterialIdx() const { return m_uiMaterialIdx; }
    const std::string& GetName() const { return m_sName; }
    bool IsValid() const { return m_bIsValid; }

    const SBoundingBox& GetLocalBounds() const { return m_localBounds; }
    const SBoundingSphere& GetBoundingSphere() const { return m_boundingSphere; }

protected:
    std::string m_sName = "Mesh_00";
    uint32_t m_uiBaseVertex = 0;
    uint32_t m_uiBaseIndex = 0;
    uint32_t m_uiIndexCount = 0;
    uint32_t m_uiVertexCount = 0;
    uint32_t m_uiMaterialIdx = 0;
    SBoundingBox m_localBounds = {};
    SBoundingSphere m_boundingSphere = {};
    bool m_bIsValid = false;
};

class CModelAssetBase
{
public:
    virtual ~CModelAssetBase() = default;

    virtual void Clear() = 0;
    virtual bool InitializeMaterialBindings() = 0;
   
    const std::vector<IMaterial*>& GetMaterials() const { return m_vMaterials; }
    std::vector<IMaterial*>& GetMaterials() { return m_vMaterials; }
    IMaterial* GetMaterial(size_t index) { return m_vMaterials.at(index); }

    const std::string& GetMeshFilePath() const { return m_stMeshFilePath; }
    void SetMeshFilePath(const std::string& stFilePath) { m_stMeshFilePath = stFilePath; }
    const std::string& GetMeshName() const { return m_stMeshName; }
    void SetMeshName(const std::string& stName) { m_stMeshName = stName; }
    EModelAssetType GetModelType() const { return m_eModelAssetType; }
    void SetModelType(const EModelAssetType eModelAssetType) { m_eModelAssetType = eModelAssetType; }

    const std::vector<SMeshBatch>& GetNewBatches() const { return m_vNewBatches; }

protected:
    // Mesh Data
    std::string m_stMeshFilePath;
    std::string m_stMeshName;
    EModelAssetType m_eModelAssetType = EModelAssetType::MODEL_TYPE_STATIC;

    // Model Materials
    std::vector<IMaterial*> m_vMaterials = {};
    std::vector<SMeshBatch> m_vNewBatches = {};
};