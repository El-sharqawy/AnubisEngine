#version 450

layout(location = 0) in vec3 m_v3Position;
layout(location = 1) in vec3 m_v3Normal;
layout(location = 2) in vec2 m_v2TexCoord;
layout(location = 3) in vec4 m_v4Tangent;

layout(location = 0) out vec2 fragTexCoord;

layout(std140, set = 1, binding = 0) uniform SCameraUBO
{
    mat4 view;
    mat4 proj;
    mat4 viewProj;
} cameraUBO;

layout(push_constant) uniform PushModel
{
    mat4 model;
    uint skinPaletteIndex;
    uint flags;
    uint bPadding[2];
} modelData;

void main()
{
    gl_Position = cameraUBO.proj * cameraUBO.view * modelData.model * vec4(m_v3Position, 1.0);
    fragTexCoord = m_v2TexCoord;
}
