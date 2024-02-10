struct IN
{
	float4 Position : SV_POSITION;
	float3 Normal : NORMAL;
};

float3 Diffuse(const float3 MaterialColor, const float3 LightColor, const float LN)
{
	return saturate(saturate(LN) * MaterialColor * LightColor);
}
float3 Blinn(const float3 MaterialColor, const float4 LightColor, const float LN, const float3 L, const float3 N, const float3 V)
{
	return saturate(saturate(sign(LN)) * pow(saturate(dot(N, normalize(V + L))), LightColor.a) * LightColor.rgb * MaterialColor);
}
float3 Phong(const float3 MaterialColor, const float4 LightColor, const float LN, const float3 L, const float3 N, const float3 V)
{
	return saturate(saturate(sign(LN)) * pow(saturate(dot(reflect(-L, N), V)), LightColor.a) * LightColor.rgb * MaterialColor);
}
float3 Specular(const float3 MaterialColor, const float4 LightColor, const float LN, const float3 L, const float3 N, const float3 V)
{
	return Phong(MaterialColor, LightColor, LN, L, N, V);
}

static const float3 LightDirection = float3(0.0f, 1.0f, 0.0f);

float4 main(IN In) : SV_TARGET
{
	const float3 N = normalize(In.Normal);
	//const float3 V = normalize(In.ViewDirection);
	const float3 L = normalize(LightDirection);
	const float LN = dot(L, N);
	const float3 Ambient = float3(0.1f, 0.1f, 0.1f);
	const float3 MaterialColor = float3(1.0f, 1.0f, 1.0f);
	const float4 LightColor = float4(1.0f, 1.0f, 1.0f, 8.0f);
	const float Attenuate = 1.0;
	const float Spot = 1.0;
	const float3 Color = (Ambient + (Diffuse(MaterialColor, LightColor.rgb, LN) /*+ Specular(MaterialColor, LightColor, LN, L, N, V)*/) * Attenuate) * Spot;


	return float4(Color, 1.0f);
}
