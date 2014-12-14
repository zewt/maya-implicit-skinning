//--------------------------------------------------------------------------------------
// Order Independent Transparency with Depth Peeling
//
// Author: Louis Bavoil
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

uniform float Alpha;

varying vec3 eye;
varying vec3 normal;
varying float depth;


vec3 ShadeVertex(){
	return gl_Vertex.xyz;
}

void main(void)
{

	const float zfar = 100.0;
	const float znear = 1.0;
	normal = normalize(gl_NormalMatrix * gl_Normal)	;
	vec4 eyecoords = gl_ModelViewMatrix * gl_Vertex;
	eye = normalize(-vec3(eyecoords));
	gl_Position = ftransform();
	depth = (zfar + znear - 2.0 * zfar * znear / (-eyecoords.z))/(zfar - znear);
	//depth = -eyecoords.z/100.0;
	//depth = gl_Position.z;
	gl_TexCoord[0].xyz = ShadeVertex();
	gl_FrontColor = gl_Color;
}
