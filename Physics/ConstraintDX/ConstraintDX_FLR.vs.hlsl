struct IN
{
	float3 Position : POSITION;
	float3 Normal : NORMAL;
	uint InstanceID : SV_InstanceID;
};

struct INSTANCE
{
	float4x4 World;
};
struct WORLD_BUFFER
{
	INSTANCE Instances0[64];
	INSTANCE Instances1[64];
};
ConstantBuffer<WORLD_BUFFER> WB : register(b0, space0);
struct VIEW_PROJECTION_BUFFER
{
	float4x4 ViewProjection;
};
ConstantBuffer<VIEW_PROJECTION_BUFFER> VPB : register(b1, space0);

struct OUT
{
	float4 Position : SV_POSITION;
	float3 Normal : NORMAL;
};

OUT main(IN In)
{
	OUT Out;

	const float4x4 W = WB.Instances1[In.InstanceID].World;
	const float4x4 WVP = mul(VPB.ViewProjection, W);

	Out.Position = mul(WVP, float4(In.Position, 1.0f));
	Out.Normal = mul((float3x3)W, In.Normal);

	return Out;
}
