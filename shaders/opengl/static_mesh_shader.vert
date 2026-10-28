#version 460

// Vertex Inputs (Compatible with both)
layout(location = 0) in vec3 m_v3Position;
layout(location = 1) in vec3 m_v3Normal;
layout(location = 2) in vec2 m_v2TexCoord;
layout(location = 3) in vec4 m_v4Tangent;

layout(location = 0) out vec2 fragTexCoord;

// --- Modern Path: Uses SSBO and gl_DrawID ---
// Camera UBO
layout(std140, binding = 0) uniform SCameraUBO
{
    mat4 View;
    mat4 Projection;
    mat4 ViewProjection;
} cameraUBO;

// Model Data (Push Constant for Vulkan, Uniform Block for OpenGL)
layout(std140, binding = 1) uniform SModelData
{
    mat4 matModel;
    uint skinPaletteFirstMatrix;
    uint skinMatrixCount;
    uint flags;
    uint bPadding[1];
} modelData;

void main()
{
    vec4 skinnedPos    = modelData.matModel * vec4(m_v3Position, 1.0);
    vec3 skinnedNormal = mat3(modelData.matModel) * m_v3Normal;

    vec4 worldPos = modelData.matModel * skinnedPos;
    gl_Position = cameraUBO.ViewProjection * worldPos;
    fragTexCoord = m_v2TexCoord;
}
