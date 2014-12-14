/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
uniform sampler2D bgl_DepthTexture;
uniform sampler2D bgl_RenderedTexture;
//uniform sampler2D bgl_LuminanceTexture; // luminance texture needed to discard ao on highlighted areas
//uniform float bgl_RenderedTextureWidth;
//uniform float bgl_RenderedTextureHeight;

#define PI    3.14159265

// TODO: set these as uniform
uniform int width;//bgl_RenderedTextureWidth; //texture width
uniform int height;//bgl_RenderedTextureHeight; //texture height
// TODO: set these as uniform
float near = 1.0; //Z-near
float far = 100.0; //Z-far

int samples = 8; //samples on the first ring
int rings = 6; //ring count

//vec2 texCoord = gl_TexCoord[0].st;
varying in vec2 uv;

vec2 rand(in vec2 coord) //generating random noise
{
    float noiseX = (fract(sin(dot(coord ,vec2(12.9898,78.233))) * 43758.5453));
    float noiseY = (fract(sin(dot(coord ,vec2(12.9898,78.233)*2.0)) * 43758.5453));
    return vec2(noiseX,noiseY)*0.004;
}

float readDepth(in vec2 coord)
{
    return (2.0 * near) / (far + near - texture2D(bgl_DepthTexture, coord ).x * (far-near));
}

float compareDepths( in float depth1, in float depth2 )
{
    float aoCap = 1.0;
    float aoMultiplier = 100.0;
    float depthTolerance = 0.0000;
    float aorange = 6.0;// units in space the AO effect extends to (this gets divided by the camera far range
    float diff = sqrt(clamp(1.0-(depth1-depth2) / (aorange/(far-near)),0.0,1.0));
    float ao = min(aoCap,max(0.0,depth1-depth2-depthTolerance) * aoMultiplier) * diff;
    return ao;
}

void main(void)
{
    float depth = readDepth(uv);
    float d;

    float aspect = (float)width/(float)height;
    vec2 noise = rand(uv);

    float w = (1.0 / (float)width)/clamp(depth,0.05,1.0)+(noise.x*(1.0-noise.x));
    float h = (1.0 / (float)height)/clamp(depth,0.05,1.0)+(noise.y*(1.0-noise.y));

    float pw;
    float ph;

    float ao;
    float s;
    float fade = 1.0;

    for (int i = 0 ; i < rings; i += 1)
    {
    fade *= 0.5;
        for (int j = 0 ; j < samples*i; j += 1)
        {
            float step = PI*2.0 / (samples*i);
            pw = (cos(j*step)*i);
            ph = (sin(j*step)*i)*aspect;
            d = readDepth( vec2(uv.s+pw*w,uv.t+ph*h));
            ao += compareDepths(depth,d)*fade;
            s += 1.0*fade;
        }
    }

    ao /= s;
    ao = 1.0-ao;

    vec3 color = texture2D(bgl_RenderedTexture,uv).rgb;
    vec3 luminance = vec3(0.0, 0.0, 0.0);//texture2D(bgl_LuminanceTexture,uv).rgb;

    luminance = clamp(max(0.0,luminance-0.2)+max(0.0,luminance-0.2)+max(0.0,luminance-0.2),0.0,1.0);

    gl_FragColor = vec4(color*mix(vec3(ao),vec3(1.0),luminance),1.0);

    //gl_FragColor = vec4(depth ,depth ,depth , 1.0);

    //gl_FragColor = vec4(color, 1.0);
    //gl_FragColor = vec4(ao, ao, ao, 1.0);
}
