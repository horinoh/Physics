#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) in vec3 InPosition;

struct INSTANCE
{
	mat4 World;
};
layout (set = 0, binding = 0) uniform WORLD_BUFFER
{
	INSTANCE Instances0[64];
	INSTANCE Instances1[64];
} WB;

layout (set = 0, binding = 1) uniform VIEW_PROJECTION_BUFFER
{
	mat4 ViewProjection;
} VPB;

void main()
{
	const mat4 PVW = VPB.ViewProjection * WB.Instances0[gl_InstanceIndex].World;

	gl_Position = PVW * vec4(InPosition, 1.0f);
}
