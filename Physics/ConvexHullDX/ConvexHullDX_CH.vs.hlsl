struct IN
{
	float3 Position : POSITION;
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
}; 

OUT main(IN In)
{
	OUT Out;

	const float4x4 WVP = mul(VPB.ViewProjection, WB.Instances0[In.InstanceID].World);

	Out.Position = mul(WVP, float4(In.Position, 1.0f));

	return Out;
}
