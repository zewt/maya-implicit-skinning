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
varying vec2 tex_coord;
varying float z_;

void main(){
  vec4 eye_space = gl_ModelViewMatrix * gl_Vertex;
	gl_Position = gl_ModelViewProjectionMatrix *  gl_Vertex;
  gl_Position.xyz /=  gl_Position.w;
	//gl_Position.z = 1.;
	gl_Position.w = 1.;
	tex_coord = gl_Position.xy * vec2(0.5,-0.5) + 0.5;
	z_ = -eye_space.z;
	gl_FrontColor = gl_Color;
}
