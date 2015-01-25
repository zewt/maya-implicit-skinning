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
//#include "textures.hpp"
#include "cuda_utils.hpp"
#include "vec3_cu.hpp"
#include "constants.hpp"

// =============================================================================
namespace Constants{
// =============================================================================

float* d_constants;
const int nb_constants = NB_CONST;
float h_constants[nb_constants];
bool binded = false;

texture<float, 1,  cudaReadModeElementType> constants_tex;

// -----------------------------------------------------------------------------

void allocate(){
    int size = nb_constants * sizeof(float);
    CUDA_SAFE_CALL(cudaMalloc((void**)(&d_constants),size));
}

void free(){
    cudaFree(d_constants);
    d_constants = 0;
}

// -----------------------------------------------------------------------------

extern void bind();
extern void unbind();

// -----------------------------------------------------------------------------

void update(){
    unbind();
    int size = nb_constants * sizeof(float);
    CUDA_SAFE_CALL(cudaMemcpy(d_constants,
                              h_constants,
                              size,
                              cudaMemcpyHostToDevice));
    bind();
}

// -----------------------------------------------------------------------------

void  set(int var, float value){ h_constants[var] = value; }
float get(int var)             { return h_constants[var];  }

// -----------------------------------------------------------------------------

void incr(int var, float value, float minv, float maxv){
    h_constants[var] += value;
    if(h_constants[var] > maxv)
        h_constants[var] = maxv;
    if(h_constants[var] < minv)
        h_constants[var] = minv;
}

// -----------------------------------------------------------------------------

void init(){
    set(B0, -1.f);
    set(B1, 0.f);
    set(B2, 1.f);
    set(F0, 1.f);
    set(F1, 0.f);
    set(F2, 0.995f);
    set(POW0, 1.f);
    set(POW1, 1.f);

    set(K0, 1.f);
    set(K1, 1.f);
    set(K2, 1.f);
    set(K3, 1.f);
    set(K4, 1.f);
    set(K5, 1.f);
}

// -----------------------------------------------------------------------------

};
// END CONSTANTS NAMESPACE =====================================================
