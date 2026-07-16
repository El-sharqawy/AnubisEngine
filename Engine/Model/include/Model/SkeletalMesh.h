#pragma once

#include <string>
#include "API/RenderObject.h"
#include "Model/SkeletalMeshData.h"
#include "BoundingBox.h"
#include <assimp/mesh.h>

class CSkeleton;

class CSkeletalMesh : public CMeshAssetBase
{
public:
    CSkeletalMesh() = default;
    ~CSkeletalMesh() = default;

    // Non-copyable — owns GPU resources
    CSkeletalMesh(const CSkeletalMesh&) = delete;
    CSkeletalMesh& operator=(const CSkeletalMesh&) = delete;

    // Movable — for std::vector<CVulkanMesh> in CModel
    CSkeletalMesh(CSkeletalMesh&& other) noexcept;
    CSkeletalMesh& operator=(CSkeletalMesh&& other) noexcept;

    bool LoadMesh(const aiMesh* pAiMesh, uint32_t uiMaterialIndex, const CSkeleton& skeleton);
    void Unload();
    void SetVertexBoneData(SSkeletalMeshVertex& vertex, int boneID, float weight);
    void ComputeBounds(const std::vector<SSkeletalMeshVertex>& vVertices);

    // Data
    const std::vector<SSkeletalMeshVertex>& GetVertices() const { return m_vVertices; }
    std::vector<SSkeletalMeshVertex>& GetVertices() { return m_vVertices; }
    const std::vector<uint32_t>& GetIndices() const { return m_vIndices; }
    std::vector<uint32_t>& GetIndices() { return m_vIndices; }

private:
    std::vector<uint32_t> m_vIndices;
    std::vector<SSkeletalMeshVertex> m_vVertices;
};
