#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (set = 0, binding = 1) uniform VIEW_PROJECTION_BUFFER
{
	mat4 ViewProjection[4];
} VPB;

layout (points, invocations = 4) in;
layout (points, max_vertices = 2) out;
void main()
{
	for(int i=0;i<gl_in.length();++i) {
		gl_Position =  VPB.ViewProjection[gl_InvocationID] * gl_in[i].gl_Position;
		gl_ViewportIndex = gl_InvocationID;
		gl_PointSize = 5.0f;
		EmitVertex();
	}
	EndPrimitive();	
}
