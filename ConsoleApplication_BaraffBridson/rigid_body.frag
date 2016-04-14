# version 330 core

struct Light
{
	vec3 direction;
	vec3 color;
};
uniform Light light;

struct Material
{
	vec3 ambient;
	vec3 diffuse;
	vec3 specular;
	float shininess;
};
uniform Material material;

uniform vec3 viewPos;

in VS_OUT
{
	vec3 normal;
	vec3 fragPos;
	vec3 position;
	float data;
} frag_in;

out vec4 color;

void main() 
{
	vec3 ambient = light.color * material.ambient;

	vec3 norm = normalize(frag_in.normal);
	vec3 diffuse = light.color * material.diffuse * max(dot(norm, light.direction), 0.0);
	
	vec3 viewDir = normalize(viewPos - frag_in.fragPos);
	vec3 reflectDir = reflect(-light.direction, norm);
	float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
	vec3 specular = light.color * material.specular * spec;

	color = vec4(ambient + diffuse + specular, 1.0f);
	
	//color = vec4(0.3f, 0.49f, 0.85f, 1.0f);
	//color = vec4(frag_in.data, frag_in.data, frag_in.data, 1.0f);
}