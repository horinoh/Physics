struct IN
{
	float3 Position : POSITION;
	float3 Normal : NORMAL;
	uint InstanceID : SV_InstanceID;
};

struct RIGID_BODY
{
	float4x4 World;
	float3 Color;
	float3 ClosestPoint;
};
struct WORLD_BUFFER
{
	RIGID_BODY RigidBodies[2];
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
	float3 Color : COLOR;
};

OUT main(IN In)
{
	OUT Out;

	const float4x4 W = WB.RigidBodies[In.InstanceID].World;
	const float4x4 WVP = mul(VPB.ViewProjection, W);

	Out.Position = mul(WVP, float4(In.Position, 1.0f));
	Out.Normal = mul(transpose((float3x3)W), In.Normal);
	Out.Color = WB.RigidBodies[In.InstanceID].Color;

	return Out;
}
