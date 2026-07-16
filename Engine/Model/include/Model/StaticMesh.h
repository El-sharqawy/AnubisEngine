#pragma once

#include <string>
#include "API/RenderObject.h"
#include "StaticMeshData.h"
#include "BoundingBox.h"
#include <assimp/mesh.h>

class CStaticMesh : public CMeshAssetBase
{
public:
    CStaticMesh() = default;
    ~CStaticMesh() = default;

    // Non-copyable — owns GPU resources
    CStaticMesh(const CStaticMesh&) = delete;
    CStaticMesh& operator=(const CStaticMesh&) = delete;

    // Movable — for std::vector<CVulkanMesh> in CModel
    CStaticMesh(CStaticMesh&& other) noexcept;
    CStaticMesh& operator=(CStaticMesh&& other) noexcept;

    bool LoadMesh(const aiMesh* pAiMesh, uint32_t uiMaterialIndex);
    void Unload();
    void ComputeBounds(const std::vector<SStaticMeshVertex>& vVertices);

    // Data
    const std::vector<SStaticMeshVertex>& GetVertices() const { return m_vVertices; }
    const std::vector<uint32_t>& GetIndices() const { return m_vIndices; }

private:
    std::vector<uint32_t> m_vIndices;
    std::vector<SStaticMeshVertex> m_vVertices;
};
