#version 460 core

layout (location = 0) in vec3 m_v3Position;
layout (location = 1) in vec2 m_v2TexCoord;
layout (location = 2) in vec3 m_v3Normals;

uniform mat4 viewProjectionMatrix;

void main()
{
	gl_Position = viewProjectionMatrix * vec4(m_v3Position, 1.0f);	
}
