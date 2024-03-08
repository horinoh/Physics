#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) in vec3 InPosition;
layout (location = 1) in vec3 InNormal;

struct INSTANCE
{
	mat4 World;
	vec3 Color;
	vec3 ClosestPoint;
};
layout (set = 0, binding = 0) uniform WORLD_BUFFER
{
	INSTANCE Instances[2];
} WB;
layout (set = 0, binding = 1) uniform VIEW_PROJECTION_BUFFER
{
	mat4 ViewProjection;
} VPB;

layout (location = 0) out vec3 OutNormal;
layout (location = 1) out vec3 OutColor;

void main()
{
	const mat4 W = WB.Instances[gl_InstanceIndex].World;
	const mat4 PVW = VPB.ViewProjection * W;

	gl_Position = PVW * vec4(InPosition, 1.0f);
	OutNormal = transpose(mat3(W)) * InNormal;
	OutColor = WB.Instances[gl_InstanceIndex].Color;
}
