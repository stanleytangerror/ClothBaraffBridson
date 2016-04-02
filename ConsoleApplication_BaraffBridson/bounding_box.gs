#version 330 core

layout (lines) in;
layout (line_strip, max_vertices = 24) out;

uniform mat4 projection;
uniform mat4 view;
uniform mat4 model;

vec3 vertices[8];
mat4 trans = projection * view * model;

void GenerateLine(int i, int j)
{
    gl_Position = trans * vec4(vertices[i], 1.0f);
    //gl_Position = vec4(vertices[i], 1.0f);
    EmitVertex();
    gl_Position = trans * vec4(vertices[j], 1.0f);
    //gl_Position = vec4(vertices[j], 1.0f);
    EmitVertex();
}

void main() 
{
	vec3 minp = gl_in[0].gl_Position.xyz;
	vec3 maxp = gl_in[1].gl_Position.xyz;
	vec3 diry = vec3(0.0f, maxp.y - minp.y, 0.0f);
	
	vertices[0] = minp;
	vertices[1] = vec3(maxp.x, minp.yz);
	vertices[2] = vec3(maxp.x, minp.y, maxp.z);
	vertices[3] = vec3(minp.xy, maxp.z);
	vertices[4] = vertices[0] + diry; 
	vertices[5] = vertices[1] + diry; 
	vertices[6] = vertices[2] + diry; 
	vertices[7] = vertices[3] + diry; 

	GenerateLine(0, 1);
	GenerateLine(1, 2);
	GenerateLine(2, 3);
	GenerateLine(3, 0);
    EndPrimitive();

	GenerateLine(4, 5);
	GenerateLine(5, 6);
	GenerateLine(6, 7);
	GenerateLine(7, 4);
    EndPrimitive();

	GenerateLine(0, 4);
    EndPrimitive();

	GenerateLine(1, 5);
    EndPrimitive();

	GenerateLine(2, 6);
    EndPrimitive();

	GenerateLine(3, 7);
    EndPrimitive();
	
}
