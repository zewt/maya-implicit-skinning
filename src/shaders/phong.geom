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
#version 120
#extension GL_EXT_geometry_shader4 : enable
#extension GL_EXT_gpu_shader4 : enable

/** Triangle subdivision */

varying in vec3 vnormal[];
varying in vec4 vposition[]; // camera space (modelview) position
varying in vec3 vhalfVec[];
varying in vec3 vlightVec[];
varying in vec3 vtangent[];
varying in vec2 vtexCoord0[];

varying out vec3 normal;
varying out vec3 halfVec;
varying out vec3 lightVec;
varying out vec3 tangent;
varying out vec2 texCoord0;

// -----------------------------------------------------------------------------

vec4 p0;
vec4 p1;
vec4 p2;

vec3 n0;
vec3 n1;
vec3 n2;

vec4 v0;
vec4 v1;

/// plane equation at each triangle vertices
/// pln.x, pln.y, pln.z = plane normal n.
/// pln.w = distance from origin (i.e: -n dot origin)
vec4 pln0;
vec4 pln1;
vec4 pln2;


bool to_flip;

// -----------------------------------------------------------------------------

// Euclidean squared distance between p and q
float dst2(vec4 p, vec4 q){
    vec3 d = (p - q).xyz;
    return dot(d,d);
}

// -----------------------------------------------------------------------------

/// Project pos on to the plane pln
vec4 project(vec4 pos, vec4 pln){
    // compute orthogonal distance from the plane pln
    float plane_dist = dot(pos,pln);
    // project the point pos on to the plane pln
    return pos + -plane_dist * vec4(pln.xyz,0);
}

// -----------------------------------------------------------------------------

void produce_vertex(float u, float v){
    vec4 pos = p0 + v0 * u + v1 * v;

    vec4 pr0 = project(pos, pln0);
    vec4 pr1 = project(pos, pln1);
    vec4 pr2 = project(pos, pln2);

    float w = 1.0 - u - v;

    vec4 npos = pr1 * u + pr2 * v  + pr0 * w;

    npos = npos;

    vec3 par = vec3(u,v,w);
    // normal   = -normalize(n1 * par.x  + n2 * par.y + n0 * par.z);
    normal   = normalize(n1 * u  + n2 * v + n0 * w);

    gl_Position = gl_ProjectionMatrix * npos;
    EmitVertex();
}

// -----------------------------------------------------------------------------

void main(){

#if 1
    // first vertex
    normal = vnormal[0];
    halfVec = vhalfVec[0];
    lightVec = vlightVec[0];
    gl_FrontColor = gl_FrontColorIn[0];
    tangent = vtangent[0];
    #ifdef TEXTURE_KD
    texCoord0 = vtexCoord0[0];
    #endif
    gl_Position = gl_PositionIn[0];
    EmitVertex();

    // second vertex
    normal = vnormal[1];
    halfVec = vhalfVec[1];
    lightVec = vlightVec[1];
    gl_FrontColor = gl_FrontColorIn[1];
    tangent = vtangent[1];
    #ifdef TEXTURE_KD
    texCoord0 = vtexCoord0[1];
    //texCoord = vtexCoord[0];
    #endif
    gl_Position = gl_PositionIn[1];
    EmitVertex();

    // third vertex
    normal = vnormal[2];
    halfVec = vhalfVec[2];
    lightVec = vlightVec[2];
    gl_FrontColor = gl_FrontColorIn[2];
    tangent = vtangent[2];
    #ifdef TEXTURE_KD
    texCoord0 = vtexCoord0[2];
    #endif
    gl_Position = gl_PositionIn[2];
    EmitVertex();
    // send triangle
    EndPrimitive();

#else
//This part of the shader needs to be re-written in order to write out
// th lightVecc texCoord etc. atributes don't forget an attribute!
    to_flip = dst2(vposition[0],vposition[1]) > dst2(vposition[1],vposition[2]);
    if(to_flip){
        p0 = vposition[2];
        p1 = vposition[1];
        p2 = vposition[0];
        n0 = vnormal[2];
        n1 = vnormal[1];
        n2 = vnormal[0];
    } else {
        p0 = vposition[0];
        p1 = vposition[1];
        p2 = vposition[2];
        n0 = vnormal[0];
        n1 = vnormal[1];
        n2 = vnormal[2];
    }4

    v0 = p1 - p0;
    v1 = p2 - p0;
    // Compute plane equation corresponding to each point
    pln0.xyz = n0;
    pln0.w = -dot(pln0.xyz, p0.xyz) * p0.w;
    pln1.xyz = n1;
    pln1.w = -dot(pln1.xyz, p1.xyz) * p1.w;
    pln2.xyz = n2;
    pln2.w = -dot(pln2.xyz, p2.xyz) * p2.w;

    // Subdivide
    int n = 4;
    float dt = 1.f / float(n);
    float u0 = 0.0;
    float u1 = dt;
    for(int i = 0; i < n; i++){
        int nb = n  - i;
        int j;
        float v = 0.0;
        for(j = 0; j < nb; j++){
            produce_vertex(u0, v);
            produce_vertex(u1, v);
            v += dt;
        }
        produce_vertex(u0, v);
        EndPrimitive();
        u0 += dt;
        u1 += dt;
    }
#endif
}

// -----------------------------------------------------------------------------
