#version 450

// 1. Cross-API Layout Macros
#ifdef VULKAN
    #define SET_BINDING(set_idx, bind_idx) layout(set = set_idx, binding = bind_idx)
    #define PUSH_CONSTANT layout(push_constant) uniform
#else // OPENGL
    // OpenGL doesn't use descriptor sets, so we ignore the set parameter
    #define SET_BINDING(set_idx, bind_idx) layout(binding = bind_idx)
    // OpenGL uses uniforms instead of push constants
    #define PUSH_CONSTANT layout(std140, binding = 3) uniform
#endif

// Vertex Inputs (Compatible with both)
layout(location = 0) in vec3 m_v3Position;
layout(location = 1) in vec3 m_v3Normal;
layout(location = 2) in vec2 m_v2TexCoord;
layout(location = 3) in vec4 m_v4Tangent;

layout(location = 0) out vec2 fragTexCoord;

// --- Modern Path: Uses SSBO and gl_DrawID ---
// Camera UBO
SET_BINDING(0, 0) uniform SCameraUBO
{
    mat4 view;
    mat4 proj;
    mat4 viewProj;
} cameraUBO;

// Model Data (Push Constant for Vulkan, Uniform Block for OpenGL)
PUSH_CONSTANT PushModel
{
    mat4 model;
} modelData;

void main()
{
    gl_Position = cameraUBO.proj * cameraUBO.view * modelData.model * vec4(m_v3Position, 1.0);
    fragTexCoord = m_v2TexCoord;
}
