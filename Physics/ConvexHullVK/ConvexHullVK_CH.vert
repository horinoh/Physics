#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) in vec3 InPosition;

struct RIGID_BODY
{
	mat4 World;
};
layout (set = 0, binding = 0) uniform WORLD_BUFFER
{
	RIGID_BODY RigidBodies[64];
} WB;

layout (set = 0, binding = 1) uniform VIEW_PROJECTION_BUFFER
{
	mat4 ViewProjection;
} VPB;

void main()
{
	const mat4 PVW = VPB.ViewProjection * WB.RigidBodies[gl_InstanceIndex].World;

	gl_Position = PVW * vec4(InPosition, 1.0f);
}
