# version 330
in vec3 TexCoords;
out vec4 color;

uniform samplerCube background;

void main()
{
	color = texture(background, TexCoords);
}