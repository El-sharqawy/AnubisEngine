#version 450

layout (location = 0) in vec2 fragTexCoord;
layout (location = 0) out vec4 outColor;

layout(binding = 0) uniform sampler2D uDiffuseMap;
layout(binding = 1) uniform sampler2D uSpecularMap;
layout(binding = 2) uniform sampler2D uAlbedoMap;
layout(binding = 3) uniform sampler2D uRoughnessMap;
layout(binding = 4) uniform sampler2D uMetallicMap;
layout(binding = 5) uniform sampler2D uNormalMap;

void main()
{
    outColor = texture(uDiffuseMap, fragTexCoord);
}
