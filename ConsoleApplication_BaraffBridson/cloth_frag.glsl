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

struct SurfaceOutputStandard
{
	vec3	Albedo;      // base (diffuse or specular) color
	vec3	Normal;      // tangent space normal, if written
	vec3	Emission;
	float	Metallic;      // 0=non-metal, 1=metal，金属度
	float	Smoothness;    // 0=rough, 1=smooth，光滑度
	float	Occlusion;     // occlusion (default 1)，遮挡率
	float	Alpha;        // alpha for transparencies
};

const float PI = 3.14159265359;

vec3 mix(vec3 v1, vec3 v2, float t)
{
	return v1 * t + v2 * (1 - t);
}

float DistributionGGX(vec3 N, vec3 H, float roughness)
{
	float a = roughness*roughness;
	float a2 = a*a;
	float NdotH = max(dot(N, H), 0.0);
	float NdotH2 = NdotH*NdotH;

	float num = a2;
	float denom = (NdotH2 * (a2 - 1.0) + 1.0);
	denom = PI * denom * denom;

	return num / denom;
}

float GeometrySchlickGGX(float NdotV, float roughness)
{
	float r = (roughness + 1.0);
	float k = (r*r) / 8.0;

	float num = NdotV;
	float denom = NdotV * (1.0 - k) + k;

	return num / denom;
}
float GeometrySmith(vec3 N, vec3 V, vec3 L, float roughness)
{
	float NdotV = max(dot(N, V), 0.0);
	float NdotL = max(dot(N, L), 0.0);
	float ggx2 = GeometrySchlickGGX(NdotV, roughness);
	float ggx1 = GeometrySchlickGGX(NdotL, roughness);

	return ggx1 * ggx2;
}
vec3 fresnelSchlick(float cosTheta, vec3 F0)
{
	return F0 + (1.0 - F0) * pow(1.0 - cosTheta, 5.0);
}

vec3 GammaCorrect(vec3 c)
{
	c = c / (c + vec3(1.0));
	return pow(c, vec3(1.0 / 2.2));
}

void main()
{
	vec3 N = normalize(-frag_in.normal);
	vec3 V = normalize(viewPos - frag_in.fragPos);

	vec3 lightPos = vec3(3);
	vec3 surfaceToLight = (lightPos - frag_in.fragPos);
	vec3 lightColor = light.color;

	///
	SurfaceOutputStandard o;
	o.Albedo = material.diffuse;
	o.Normal = N;
	o.Emission = vec3(0);
	o.Metallic = 0;
	o.Smoothness = 0.5;
	o.Occlusion = 1.0;
	o.Alpha = 1.0;
	///

	vec3 albedo = o.Albedo;
	float metallic = o.Metallic;
	float roughness = clamp(1 - o.Smoothness, 0, 1);
	float ao = o.Occlusion;

	vec3 F0 = vec3(0.04);
	F0 = mix(F0, albedo, metallic);

	// reflectance equation
	vec3 Lo = vec3(0.0);
	{
		// calculate per-light radiance
		vec3 L = normalize(surfaceToLight);
		vec3 H = normalize(V + L);
		float distance = length(surfaceToLight); distance = 1;
		float attenuation = 1.0 / (distance * distance);
		vec3 radiance = lightColor * attenuation;

		// cook-torrance brdf
		float NDF = DistributionGGX(N, H, roughness);
		float G = GeometrySmith(N, V, L, roughness);
		vec3 F = fresnelSchlick(max(dot(H, V), 0.0), F0);

		vec3 kS = F;
		vec3 kD = vec3(1.0) - kS;
		kD *= 1.0 - metallic;

		vec3 numerator = NDF * G * F;
		float denominator = 4.0 * max(dot(N, V), 0.0) * max(dot(N, L), 0.0);
		vec3 specular = numerator / max(denominator, 0.001);

		// add to outgoing radiance Lo
		float NdotL = max(dot(N, L), 0.0);
		Lo += (kD * albedo / PI + specular) * radiance * NdotL;
	}

	vec3 ambient = vec3(0.2) * albedo * ao;
	vec3 realColor = ambient + Lo;

	color = vec4(GammaCorrect(realColor), 1.0);
}

#if NOT_DEFINED
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
#endif