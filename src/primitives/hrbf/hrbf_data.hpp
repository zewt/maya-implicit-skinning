#ifndef HRBF_DATA_HPP
#define HRBF_DATA_HPP

#include "vec3_cu.hpp"

/**
    Data structure to communicate with RBF between nvcc and gcc.
    The RBF wrapper has to utilize data structure which is both compatible
    with gcc and nvcc.
*/

// =============================================================================
namespace HRBF_wrapper {
// =============================================================================

    /// HermiteRbfReconstruction data wrapper in order to store RBF coeffs
    ///	for post evaluation of the potential field
    struct HRBF_coeffs {

        float*   alphas;
        Vec3_cu* nodeCenters;
        Vec3_cu* normals;
        Vec3_cu* betas;
        int size;        ///< size of the previous arrays

        ~HRBF_coeffs (){
            delete[] alphas;
            delete[] nodeCenters;
            delete[] normals;
            delete[] betas;
        }
    };

}// END RBF_wrapper ============================================================

#endif // HRBF_DATA_HPP
