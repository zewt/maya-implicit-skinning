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
const float far  = 100.;
const float near = 1.;

#if 0
varying in vec3 normal;



void main(void)
{
    vec3 normal_n = normalize(normal);
    float depth   = gl_FragCoord.z;//(2.0 * near) / (far + near - gl_FragCoord.z * (far-near));
    gl_FragColor  = vec4(normal_n, depth);
}
#endif



varying vec3 Normal;
varying float depth;
void main( void )
{
    //float depth_   = (2.0 * near) / (far + near - gl_FragCoord.z * (far-near));
   gl_FragColor = vec4(normalize(Normal),depth);
   //gl_FragColor = vec4(depth,depth,depth, 1.0);
}
