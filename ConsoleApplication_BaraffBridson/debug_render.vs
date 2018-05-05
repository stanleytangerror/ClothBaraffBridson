#version 330 core
layout (location = 0) in vec3 worldPosition;

uniform mat4 view;
uniform mat4 projection;

void main()
{
    gl_Position = projection * view * vec4(worldPosition, 1.0f);
}