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

//varying in vec3 vnormal[];
//varying in vec4 vposition[];

//varying out vec3 normal;
//varying out vec4 position;

float point_size = 0.2f;

void main()
{
    gl_TexCoord[0] = vec4(0., 1., 0., 1.);
    //normal = vnormal[0];
    gl_Position = gl_PositionIn[0] + vec4(-point_size, -point_size, 0., 0.);
    EmitVertex();
    gl_TexCoord[0] = vec4(0., 0., 0., 1.);
    //normal = vnormal[0];
    gl_Position = gl_PositionIn[0] + vec4(-point_size,  point_size, 0., 0.);
    EmitVertex();
    gl_TexCoord[0] = vec4(1., 1., 0., 1.);
    //normal = vnormal[0];
    gl_Position = gl_PositionIn[0] + vec4( point_size, -point_size, 0., 0.);
    EmitVertex();
    gl_TexCoord[0] = vec4(1., 0., 0., 1.);
    //normal = vnormal[0];
    gl_Position = gl_PositionIn[0] + vec4( point_size,  point_size, 0., 0.);
    EmitVertex();
    EndPrimitive();
}
