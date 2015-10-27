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
