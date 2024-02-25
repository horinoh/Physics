#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) in vec3 InPosition;

struct RIGID_BODY
{
	mat4 World;
	vec3 Color;
	vec3 ClosestPoint;
};
layout (set = 0, binding = 0) uniform WORLD_BUFFER
{
	RIGID_BODY RigidBodies[2];
} WB;
layout (set = 0, binding = 1) uniform VIEW_PROJECTION_BUFFER
{
	mat4 ViewProjection;
} VPB;

void main()
{
	gl_Position = VPB.ViewProjection * vec4(WB.RigidBodies[gl_InstanceIndex].ClosestPoint, 1.0f);
	gl_PointSize = 5.0f;
}
