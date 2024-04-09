#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) in vec3 InPosition;

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

void main()
{
	gl_Position = vec4(WB.Instances[gl_InstanceIndex].ClosestPoint, 1.0f);
}
