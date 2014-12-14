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
// =============================================================================
namespace Constants {
// =============================================================================

extern float* d_constants;
extern const int nb_constants;
extern float h_constants[];
extern bool binded;

__device__
inline float fetch(int var){
    return tex1Dfetch(constants_tex,var);
}

void bind(){
    if(!binded){
        constants_tex.addressMode[0] = cudaAddressModeWrap;
        constants_tex.addressMode[1] = cudaAddressModeWrap;
        constants_tex.filterMode = cudaFilterModePoint;
        constants_tex.normalized = false;
        int size = NB_CONST * sizeof(float);
        CUDA_SAFE_CALL(cudaBindTexture(0, constants_tex, d_constants, size));
        binded = true;
    }
}

void unbind(){
    if(binded){
        CUDA_SAFE_CALL(cudaUnbindTexture(constants_tex));
        binded = false;
    }
}

}// END NAMESPACE CONSTANT =====================================================
