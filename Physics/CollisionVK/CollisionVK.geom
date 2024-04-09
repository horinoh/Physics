#version 450
#extension GL_ARB_separate_shader_objects : enable
#extension GL_ARB_shading_language_420pack : enable

layout (location = 0) in vec3 InColor[];

layout (set = 0, binding = 1) uniform VIEW_PROJECTION_BUFFER
{
	mat4 ViewProjection[4];
} VPB;

layout (location = 0) out vec3 OutColor;

layout (triangles, invocations = 4) in;
layout (line_strip, max_vertices = 3) out;
void main()
{
	for(int i=0;i<gl_in.length();++i) {
		gl_Position = VPB.ViewProjection[gl_InvocationID] * gl_in[i].gl_Position;
		OutColor = InColor[i];
		gl_ViewportIndex = gl_InvocationID;
		EmitVertex();
	}
	EndPrimitive();	
}
