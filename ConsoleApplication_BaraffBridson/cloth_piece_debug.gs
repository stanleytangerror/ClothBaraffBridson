#version 330 core

layout (points) in;
layout (line_strip, max_vertices = 2) out;

in VS_OUT {
	//vec4 position;
    vec3 normal;
} gs_in[];

const float MAGNITUDE = 0.02f;

void GenerateLine(int index)
{
    gl_Position = gl_in[index].gl_Position + vec4(gs_in[index].normal, 0.0f) * MAGNITUDE * 0.02f;
    EmitVertex();
    gl_Position = gl_in[index].gl_Position + vec4(gs_in[index].normal, 0.0f) * MAGNITUDE;
    EmitVertex();
    EndPrimitive();
}

void main() 
{
    GenerateLine(0);
}