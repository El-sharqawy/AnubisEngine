#version 450

layout(location = 0) in vec3 m_v3Position;
layout(location = 1) in vec3 m_v3Normal;
layout(location = 2) in vec2 m_v2TexCoord;
layout(location = 3) in vec4 m_v4Tangent;
layout(location = 4) in vec3 m_v3Bitangent;
layout(location = 5) in vec4 m_v4Color;
layout(location = 6) in ivec4 m_iv4BoneIndices;
layout(location = 7) in vec4 m_v4BoneWeights;

layout(location = 0) out vec2 fragTexCoord;

layout(std140, binding = 0) uniform SCameraUBO
{
    mat4 View;
    mat4 Projection;
    mat4 ViewProjection;
} cameraUBO;

layout(std430, binding = 1) readonly buffer SJointBuffer
{
    mat4 matFinalBoneMatrices[];
    // Offset 0 to 6400
    // No explicit padding array is required in GLSL because std430 
    // automatically aligns the end of the block to a vec4 boundary.
} jointBufferSSBO;

layout(std140, binding = 2) uniform SModelData
{
    mat4 matModel;
    uint skinPaletteIndex;
    uint flags;
    uint bPadding[2];
} modelData;

void main()
{
    vec4 localPos = vec4(m_v3Position, 1.0);
    vec3 localNormal = m_v3Normal;
    vec3 localTangent = vec3(m_v4Tangent);

    if (modelData.flags >= 1)
    {
        mat4 skinMat = jointBufferSSBO.matFinalBoneMatrices[modelData.skinPaletteIndex + m_iv4BoneIndices.x] * m_v4BoneWeights.x + 
            jointBufferSSBO.matFinalBoneMatrices[modelData.skinPaletteIndex + m_iv4BoneIndices.y] * m_v4BoneWeights.y + 
            jointBufferSSBO.matFinalBoneMatrices[modelData.skinPaletteIndex + m_iv4BoneIndices.z] * m_v4BoneWeights.z + 
            jointBufferSSBO.matFinalBoneMatrices[modelData.skinPaletteIndex + m_iv4BoneIndices.w] * m_v4BoneWeights.w;

        localPos = skinMat * localPos;
        localNormal = mat3(skinMat) * localNormal;
        localTangent = mat3(skinMat) * localTangent;
    }

    vec4 worldPos = modelData.matModel * localPos;
    gl_Position = cameraUBO.ViewProjection * worldPos;
    fragTexCoord = m_v2TexCoord;
}
