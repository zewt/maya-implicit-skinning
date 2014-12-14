//--------------------------------------------------------------------------------------
// Order Independent Transparency with Depth Peeling
//
// Author: Louis Bavoil
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
//--------------------------------------------------------------------------------------

varying vec3 normal;
varying vec3 eye;

vec3 ShadeVertex(){
	return vec3(gl_Vertex.xyz);
}

void main(void)
{
  normal = normalize(gl_NormalMatrix * gl_Normal);
  eye = normalize(-vec3(gl_ModelViewMatrix * gl_Vertex));
  gl_Position = ftransform();
  // float far = 100.0;
  //float near = 1.0;
  //float dz = (- 2.0 * far * near )/((far - near)*gl_Position.z - far -near);
  //gl_Position.z = (dz - near) * 2.f /(far - near) - 1.f;
  gl_TexCoord[0].xyz = ShadeVertex();
  gl_FrontColor = gl_Color;
}
