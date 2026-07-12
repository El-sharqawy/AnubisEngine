#include "VulkanModel/StaticMesh.h"
#include "EngineMathVectors.h"

CStaticMesh::CStaticMesh(CStaticMesh&& other) noexcept
    : m_sName(std::move(other.m_sName))
    , m_uiBaseVertex(std::exchange(other.m_uiBaseVertex, 0))
    , m_uiBaseIndex(std::exchange(other.m_uiBaseIndex, 0))
    , m_uiIndexCount(std::exchange(other.m_uiIndexCount, 0))
    , m_uiVertexCount(std::exchange(other.m_uiVertexCount, 0))
    , m_uiMaterialIdx(std::exchange(other.m_uiMaterialIdx, 0))
    , m_localBounds(std::move(other.m_localBounds))
    , m_bIsValid(std::exchange(other.m_bIsValid, false))
{
}

CStaticMesh& CStaticMesh::operator=(CStaticMesh&& other) noexcept
{
    if (this == &other)
    {
        return *this;
    }

    // Free what THIS currently owns before overwriting:
    Unload();

    m_sName = std::move(other.m_sName);
    m_uiBaseVertex = std::exchange(other.m_uiBaseVertex, 0);
    m_uiBaseIndex = std::exchange(other.m_uiBaseIndex, 0);
    m_uiIndexCount = std::exchange(other.m_uiIndexCount, 0);
    m_uiVertexCount = std::exchange(other.m_uiVertexCount, 0);
    m_uiMaterialIdx = std::exchange(other.m_uiMaterialIdx, 0);
    m_localBounds = std::move(other.m_localBounds);
    m_bIsValid = std::exchange(other.m_bIsValid, false);

    return *this;
}

bool CStaticMesh::LoadMesh(const aiMesh* pAiMesh, uint32_t uiMaterialIndex)
{
    m_vVertices.reserve(pAiMesh->mNumVertices);
    m_vIndices.reserve(pAiMesh->mNumFaces * 3);

    const aiVector3D ZeroVec(0.0f, 0.0f, 0.0f);

    // Populate the vertex attribute vectors
    SStaticMeshVertex vertex{};

    for (int32_t i = 0; i < pAiMesh->mNumVertices; i++)
    {
        const aiVector3D& v3Pos = pAiMesh->mVertices[i];
        vertex.position = SVector3Df(v3Pos.x, v3Pos.y, v3Pos.z);

        if (pAiMesh->HasNormals())
        {
            const aiVector3D& v3Normals = pAiMesh->mNormals[i];
            vertex.normal = SVector3Df(v3Normals.x, v3Normals.y, v3Normals.z);
        }
        else
        {
            vertex.normal = SVector3Df(0.0f, 1.0f, 0.0f);
        }

        const aiVector3D& v3TexCoords = pAiMesh->HasTextureCoords(0) ? pAiMesh->mTextureCoords[0][i] : ZeroVec;
        vertex.texCoord = SVector2Df(v3TexCoords.x, 1.0f - v3TexCoords.y);

        if (pAiMesh->HasTangentsAndBitangents())
        {
            SVector3Df normal(
                pAiMesh->mNormals[i].x,
                pAiMesh->mNormals[i].y,
                pAiMesh->mNormals[i].z);

            SVector3Df tangent(
                pAiMesh->mTangents[i].x,
                pAiMesh->mTangents[i].y,
                pAiMesh->mTangents[i].z);

            SVector3Df bitangent(
                pAiMesh->mBitangents[i].x,
                pAiMesh->mBitangents[i].y,
                pAiMesh->mBitangents[i].z);

            float handedness = EngineMath::Dot(EngineMath::Cross(normal, tangent), bitangent) < 0.0f ? -1.0f : 1.0f;

            vertex.tangent = SVector4Df(tangent.x, tangent.y, tangent.z, handedness);
        }
        else
        {
            vertex.tangent = SVector4Df(1.0f, 0.0f, 0.0f, 1.0f);
        }

        m_vVertices.emplace_back(vertex);
    }

    // Populate the index buffer
    for (uint32_t i = 0; i < pAiMesh->mNumFaces; i++)
    {
        const aiFace& face = pAiMesh->mFaces[i];

        if (face.mNumIndices != 3)
        {
            continue;
        }

        m_vIndices.emplace_back(face.mIndices[0]);
        m_vIndices.emplace_back(face.mIndices[1]);
        m_vIndices.emplace_back(face.mIndices[2]);
    }

    m_uiBaseIndex = 0;
    m_uiBaseVertex = 0;

    m_uiIndexCount = static_cast<uint32_t>(m_vIndices.size());
    m_uiVertexCount = static_cast<uint32_t>(m_vVertices.size());

    m_uiMaterialIdx = uiMaterialIndex;

    m_bIsValid = true;
    return (true);
}

void CStaticMesh::Unload()
{
    m_vVertices.clear();
    m_vIndices.clear();

    m_uiBaseVertex = 0;
    m_uiBaseIndex = 0;
    m_uiIndexCount = 0;
    m_uiVertexCount = 0;
    m_uiMaterialIdx = 0;

    m_bIsValid = false;
}

