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
uniform sampler2D tex_sampler2;

float fabs(float a){
  return (a>0)?a:-a;
}

void main(){
  //gl_FragColor = vec4(tex_coord,0.,0.);
  vec4 col = texture2D(tex_sampler2,tex_coord);
  //gl_FragColor = col;
  col *= 255.;
	float z_buf = col.r + col.g * 256. + col.b * 65536.;
  int zz = (int)z_;
	vec4 to_col = vec4(zz%256,(zz/256)%256,(zz/65536)%256,0.)/256;
  float dif = (float)(z_ - z_buf);
	to_col = gl_Color;
  gl_FragColor = (dif<0)?vec4(to_col.xyz, 0.5):vec4(to_col.xyz, 0.02);
}
