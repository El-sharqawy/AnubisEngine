#pragma once

#include <string>
#include "StaticMeshData.h"
#include "BoundingBox.h"
#include <assimp/mesh.h>

class CStaticMesh
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
    bool IsValid() const { return m_bIsValid; }

    // Data
    uint32_t GetBaseVertex()  const { return m_uiBaseVertex; }
    uint32_t GetBaseIndex()   const { return m_uiBaseIndex; }
    uint32_t GetIndexCount()  const { return m_uiIndexCount; }
    uint32_t GetVertexCount() const { return m_uiVertexCount; }
    uint32_t GetMaterialIdx() const { return m_uiMaterialIdx; }

    const std::string& GetName() const { return m_sName; }
    const std::vector<SStaticMeshVertex>& GetVertices() const { return m_vVertices; }
    const std::vector<uint32_t>& GetIndices() const { return m_vIndices; }

private:
    // Identity
    std::string m_sName = "Mesh_00";

    // Global Buffer Offsets
    uint32_t m_uiBaseVertex = 0; // offset into global buffer
    uint32_t m_uiBaseIndex = 0; // offset into global buffer
    uint32_t m_uiIndexCount = 0;
    uint32_t m_uiVertexCount = 0;

    // Material
    uint32_t m_uiMaterialIdx = 0;

    // Bounds
    SBoundingBox m_localBounds = {};
    SBoundingSphere m_boundingSphere = {};

    // State
    bool m_bIsValid = false;

    std::vector<uint32_t> m_vIndices;
    std::vector<SStaticMeshVertex> m_vVertices;
};
