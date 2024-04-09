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

layout (location = 0) out vec3 OutColor;

void main()
{
	gl_Position = WB.Instances[gl_InstanceIndex].World * vec4(InPosition, 1.0f);
	OutColor = WB.Instances[gl_InstanceIndex].Color;
}
