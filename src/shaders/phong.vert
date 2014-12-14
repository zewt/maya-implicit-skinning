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
varying out vec3 vnormal;
varying out vec4 vposition;
varying out vec3 vhalfVec;
varying out vec3 vlightVec;
varying out vec3 vtangent;
varying out vec2 vtexCoord0;

attribute vec3 attr_Tangent;

void main(void)
{
#ifdef TEXTURE_KD
    vtexCoord0 = /*gl_TextureMatrix[0] **/ gl_MultiTexCoord0;
#endif
    // compute the coordinates of the vertex in camera space, in vposition
    // for the geometry shader
    vposition = gl_ModelViewMatrix * gl_Vertex;
    // compute the coordinates in clip space in gl_Position
    gl_Position   = ftransform();
    gl_FrontColor = gl_Color;

#if 0
    gl_Position.x = gl_MultiTexCoord0.x * 2.f - 1.f;
    gl_Position.y = gl_MultiTexCoord0.y * 2.f - 1.f;
    gl_Position.z = 0.f;
    gl_Position.w = 1.f;
#endif

    // compute the normal, the light vector and the half vector in camera space
    vnormal   = normalize( gl_NormalMatrix * gl_Normal );
    vlightVec = normalize( vec3(gl_LightSource[0].position - vposition) );
    vhalfVec  = normalize( normalize(-vposition.xyz) + vlightVec);

#ifdef TEXTURE_BUMP
    vtangent = normalize(gl_NormalMatrix * attr_Tangent);
    vec3 bitangent = cross(vnormal, vtangent);

    vlightVec.x = dot(vlightVec, vtangent );
    vlightVec.y = dot(vlightVec, bitangent);
    vlightVec.z = dot(vlightVec, vnormal  );
    vlightVec = normalize(vlightVec);

    vhalfVec.x = dot(vhalfVec, vtangent );
    vhalfVec.y = dot(vhalfVec, bitangent);
    vhalfVec.z = dot(vhalfVec, vnormal  );
    vhalfVec = normalize(vhalfVec);
#endif
}

