# version 330

layout (location = 0) in vec3 position;
layout (location = 1) in vec3 normal;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

out VS_OUT
{
	vec3 normal;
} vs_out;

void main()
{
	gl_Position = projection * view * model * vec4(position, 1.0f);
	   
	mat3 normalMatrix = mat3(transpose(inverse(view * model)));
    vs_out.normal = vec3(projection * vec4(normalMatrix * normal, 1.0));

}