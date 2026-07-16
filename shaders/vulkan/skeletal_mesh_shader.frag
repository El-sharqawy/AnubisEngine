#version 450

layout (location = 0) in vec2 fragTexCoord;
layout (location = 0) out vec4 outColor;

layout(set = 0, binding = 0) uniform sampler2D uDiffuseMap;
layout(set = 0, binding = 1) uniform sampler2D uSpecularMap;
layout(set = 0, binding = 2) uniform sampler2D uAlbedoMap;
layout(set = 0, binding = 3) uniform sampler2D uRoughnessMap;
layout(set = 0, binding = 4) uniform sampler2D uMetallicMap;
layout(set = 0, binding = 5) uniform sampler2D uNormalMap;

void main()
{
    outColor = texture(uDiffuseMap, fragTexCoord);
}
