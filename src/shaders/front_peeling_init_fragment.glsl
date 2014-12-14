//--------------------------------------------------------------------------------------
// Order Independent Transparency with Depth Peeling
//
// Author: Louis Bavoil
// Email: sdkfeedback@nvidia.com
//
// Copyright (c) NVIDIA Corporation. All rights reserved.
//--------------------------------------------------------------------------------------


uniform samplerRECT BaseDepthTex;

uniform float base_alpha;

varying vec3 normal;
varying vec3 eye;
varying float depth;

#define COLOR_FREQ 3.0
#define ALPHA_FREQ 3.0

#if 1
vec4 ShadeFragment()
{
	float xWorldPos = gl_TexCoord[0].x;
	float yWorldPos = gl_TexCoord[0].y;
	float diffuse = abs(normal.z);

	vec4 color;
	color = gl_Color;
	if(color.a < 0.995){
	  color.rgb *= diffuse;
  } 
	return color;
}
#else
vec4 ShadeFragment()
{
	vec4 color = gl_Color;
	if(color.a < 0.995){
		float cost = abs(dot(eye, normal));
		float diffuse = abs(normal.z);
		float f0 = 0.35;
		float f1 = 0.4;
		float a = 1.0 / (f1 - f0);
		float b = - a * f0;
		float ft = a * cost + b;
		/*color.rgb *= diffuse;
		if(cost < f0){
			color.rgb *= 0.0;
			color.a = 1.0;
		} else {
			if(cost < f1){
				color.rgb *= ft;
				color.a = ft * (color.a - 1.0) + 1.0;
			}
			}*/
		color.rgb *= cost;
	}
	return color;
}
#endif


void main(void)
{
	vec4 color = ShadeFragment();
	if(color.a < 0.995){
		color.a *= base_alpha;
	}
	float base_depth = textureRect(BaseDepthTex, gl_FragCoord.xy).r;
	if (gl_FragCoord.z >= base_depth) {
		discard;
	}
	

	/*	color.r = depth * 0.5 + 0.5;
	color.g = 0.0;
	color.b = 0.0;
	*/

	gl_FragColor = vec4(color.rgb * color.a, 1.0 - color.a);
}
