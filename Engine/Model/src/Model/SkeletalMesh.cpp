#include "Model/SkeletalMesh.h"
#include "Model/Skeleton.h"
#include "EngineMathVectors.h"
#include "Logging/LogManager.h"

CSkeletalMesh::CSkeletalMesh(CSkeletalMesh&& other) noexcept
{
    m_sName = std::move(other.m_sName);
    m_uiBaseVertex = std::exchange(other.m_uiBaseVertex, 0);
    m_uiBaseIndex = std::exchange(other.m_uiBaseIndex, 0);
    m_uiIndexCount = std::exchange(other.m_uiIndexCount, 0);
    m_uiVertexCount = std::exchange(other.m_uiVertexCount, 0);
    m_uiMaterialIdx = std::exchange(other.m_uiMaterialIdx, 0);
    m_localBounds = std::move(other.m_localBounds);
    m_bIsValid = std::exchange(other.m_bIsValid, false);
}

CSkeletalMesh& CSkeletalMesh::operator=(CSkeletalMesh&& other) noexcept
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

bool CSkeletalMesh::LoadMesh(const aiMesh* pAiMesh, uint32_t uiMaterialIndex, const CSkeleton& skeleton)
{
    m_vVertices.reserve(pAiMesh->mNumVertices);
    m_vIndices.reserve(pAiMesh->mNumFaces * 3);

    const aiVector3D ZeroVec(0.0f, 0.0f, 0.0f);

    // Populate the vertex attribute vectors
    SSkeletalMeshVertex vertex{};

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

            vertex.bitangent.x = pAiMesh->mBitangents[i].x;
            vertex.bitangent.y = pAiMesh->mBitangents[i].y;
            vertex.bitangent.z = pAiMesh->mBitangents[i].z;
        }
        else
        {
            vertex.tangent = SVector4Df(1.0f, 0.0f, 0.0f, 1.0f);
            vertex.bitangent = SVector3Df(0.0f, 0.0f, 0.0f);
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

    for (uint32_t i = 0; i < pAiMesh->mNumVertices; ++i)
    {
        for (int32_t j = 0; j < 4; ++j)
        {
            m_vVertices[i].boneIndices[j] = -1;
            m_vVertices[i].boneWeights[j] = 0.0f;
        }
    }

    const auto& boneInfoMap = skeleton.GetBoneInfoMap();
    for (uint32_t iBone = 0; iBone < pAiMesh->mNumBones; ++iBone)
    {
        const aiBone* pAiBone = pAiMesh->mBones[iBone];
        std::string boneName = pAiBone->mName.C_Str();

        auto it = boneInfoMap.find(boneName);
        assert(it != boneInfoMap.end());

        int32_t boneID = it->second.iBoneID;

        for (uint32_t iWeight = 0; iWeight < pAiBone->mNumWeights; ++iWeight)
        {
            uint32_t vertexId = pAiBone->mWeights[iWeight].mVertexId;
            float weight = pAiBone->mWeights[iWeight].mWeight;

            assert(vertexId < m_vVertices.size());
            SetVertexBoneData(m_vVertices[vertexId], boneID, weight);
        }
    }

    ComputeBounds(m_vVertices);

    m_uiBaseIndex = 0;
    m_uiBaseVertex = 0;

    m_uiIndexCount = static_cast<uint32_t>(m_vIndices.size());
    m_uiVertexCount = static_cast<uint32_t>(m_vVertices.size());

    m_uiMaterialIdx = uiMaterialIndex;

    m_bIsValid = true;
    return (true);
}

void CSkeletalMesh::Unload()
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

void CSkeletalMesh::SetVertexBoneData(SSkeletalMeshVertex& vertex, int boneID, float weight)
{
    for (int i = 0; i < MAX_BONE_INFLUENCE; ++i)
    {
        if (vertex.boneIndices[i] < 0)
        {
            vertex.boneWeights[i] = weight;
            vertex.boneIndices[i] = boneID;
            break;
        }
    }
}

void CSkeletalMesh::ComputeBounds(const std::vector<SSkeletalMeshVertex>& vVertices)
{
    if (vVertices.empty())
    {
        return;
    }

    // 1. Compute AABB
    SVector3Df min = vVertices[0].position;
    SVector3Df max = vVertices[0].position;

    for (size_t i = 1; i < vVertices.size(); ++i)
    {
        const SVector3Df& pos = vVertices[i].position;

        min.x = std::fmin(min.x, pos.x);
        min.y = std::fmin(min.y, pos.y);
        min.z = std::fmin(min.z, pos.z);

        max.x = std::fmax(max.x, pos.x);
        max.y = std::fmax(max.y, pos.y);
        max.z = std::fmax(max.z, pos.z);
    }

    m_localBounds.v3Min = min;
    m_localBounds.v3Max = max;

    // 2. Compute bounding sphere (center = AABB center, radius = max distance)
    SVector3Df center = (min + max) * 0.5f;
    float maxRadius = 0.0f; // Changed name to reflect actual distance

    for (auto& vertex : vVertices)
    {
        SVector3Df pos = SVector3Df(vertex.position.x, vertex.position.y, vertex.position.z);
        float dist = pos.distance(center);
        maxRadius = std::fmax(maxRadius, dist); // Storing actual max distance
    }

    // Later Apply Sphere
    m_boundingSphere.v3Center = center;
    m_boundingSphere.fRadius = maxRadius;
}
