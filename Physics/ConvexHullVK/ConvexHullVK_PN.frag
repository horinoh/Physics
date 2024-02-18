#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (early_fragment_tests) in;

layout (location = 0) in vec3 InNormal;
layout (location = 1) in vec3 InColor;

layout (location = 0) out vec4 OutColor;

vec3 Diffuse(const vec3 MaterialColor, const vec3 LightColor, const float LN)
{
	return clamp(clamp(LN, 0.0f, 1.0f) * MaterialColor * LightColor, 0.0f, 1.0f);
}
vec3 Blinn(const vec3 MaterialColor, const vec4 LightColor, const float LN, const vec3 L, const vec3 N, const vec3 V)
{
	return clamp(clamp(sign(LN), 0.0f, 1.0f) * pow(clamp(dot(N, normalize(V + L)), 0.0f, 1.0f), LightColor.a) * LightColor.rgb * MaterialColor, 0.0f, 1.0f);
}
vec3 Phong(const vec3 MaterialColor, const vec4 LightColor, const float LN, const vec3 L, const vec3 N, const vec3 V)
{
	return clamp(clamp(sign(LN), 0.0f, 1.0f) * pow(clamp(dot(reflect(-L, N), V), 0.0f, 1.0f), LightColor.a) * LightColor.rgb * MaterialColor, 0.0f, 1.0f);
}
vec3 Specular(const vec3 MaterialColor, const vec4 LightColor, const float LN, const vec3 L, const vec3 N, const vec3 V)
{
	return Phong(MaterialColor, LightColor, LN, L, N, V);
}

const vec3 LightDirection = vec3(0.0f, 1.0f, 1.0f);

void main()
{
	const vec3 N = normalize(InNormal);
	const vec3 L = normalize(LightDirection);
	const float LN = dot(L, N);
	const vec3 Ambient = vec3(0.1f, 0.1f, 0.1f);
	const vec3 MaterialColor = vec3(0.75f, 0.75f, 0.75f);
	const vec4 LightColor = vec4(1.0f, 1.0f, 1.0f, 8.0f);
	const float Attenuate = 1.0;
	const float Spot = 1.0;
	const vec3 Color = (Ambient + (Diffuse(MaterialColor, LightColor.rgb, LN)) * Attenuate) * Spot;

	OutColor = vec4(Color * InColor, 1.0f);
}