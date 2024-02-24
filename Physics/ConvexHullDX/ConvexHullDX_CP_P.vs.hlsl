struct IN
{
	float3 Position : POSITION;
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
	//float PSize : PSIZE; //!< DX12 �ł͋@�\���Ȃ�
};

OUT main(IN In)
{
	OUT Out;

	Out.Position = mul(VPB.ViewProjection, float4(WB.RigidBodies[In.InstanceID].ClosestPoint, 1.0f));
	//Out.PSize = 5.0f;

	return Out;
}
