#version 450

#define RENDER_ITEM_SKINNED (1u << 0)

layout(location = 0) in vec3 m_v3Position;
layout(location = 1) in vec3 m_v3Normal;
layout(location = 2) in vec2 m_v2TexCoord;
layout(location = 3) in vec4 m_v4Tangent;
layout(location = 4) in vec3 m_v3Bitangent;
layout(location = 5) in vec4 m_v4Color;
layout(location = 6) in ivec4 m_iv4BoneIndices;
layout(location = 7) in vec4 m_v4BoneWeights;

layout(location = 0) out vec2 fragTexCoord;

layout(std140, set = 1, binding = 0) uniform SCameraUBO
{
    mat4 View;
    mat4 Projection;
    mat4 ViewProjection;
} cameraUBO;

layout(std430, set = 2, binding = 0) readonly buffer SJointBuffer
{
    mat4 boneMatrices[];
    // Offset 0 to 6400
    // No explicit padding array is required in GLSL because std430 
    // automatically aligns the end of the block to a vec4 boundary.
} bones;

layout(push_constant) uniform PushModel
{
    mat4 matModel;
    uint skinPaletteFirstMatrix;
    uint skinMatrixCount;
    uint flags;
    uint bPadding[1];
} modelData;

mat4 GetSkinMatrix()
{
    if ((modelData.flags & RENDER_ITEM_SKINNED) == 0u)
    {
        return mat4(1.0);
    }

    mat4 skinMatrix =
        bones.boneMatrices[modelData.skinPaletteFirstMatrix + m_iv4BoneIndices.x] * m_v4BoneWeights.x +
        bones.boneMatrices[modelData.skinPaletteFirstMatrix + m_iv4BoneIndices.y] * m_v4BoneWeights.y +
        bones.boneMatrices[modelData.skinPaletteFirstMatrix + m_iv4BoneIndices.z] * m_v4BoneWeights.z +
        bones.boneMatrices[modelData.skinPaletteFirstMatrix + m_iv4BoneIndices.w] * m_v4BoneWeights.w;

    return skinMatrix;
}

void main()
{
    mat4 matSkin = GetSkinMatrix();

    vec4 skinnedPos    = matSkin * vec4(m_v3Position, 1.0);
    vec3 skinnedNormal = mat3(matSkin) * m_v3Normal;

    vec4 worldPos = modelData.matModel * skinnedPos;
    gl_Position = cameraUBO.ViewProjection * worldPos;
    fragTexCoord = m_v2TexCoord;
}
