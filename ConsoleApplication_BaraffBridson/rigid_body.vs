# version 330 core

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;
layout (location = 2) in float data;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out VS_OUT
{
	vec3 normal;
	vec3 fragPos;
	vec3 position;
	float data;
} vs_out;

void main()
{
	gl_Position = projection * view * model * vec4(position, 1.0f);
	
	vs_out.normal = - normal;
	vs_out.fragPos = vec3(model * vec4(position, 1.0f));
	vs_out.data = data;
}