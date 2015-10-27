#ifndef HRBF_SETUP_HPP_
#define HRBF_SETUP_HPP_

#include "hrbf_phi_funcs.hpp"

// =============================================================================
namespace HRBF_wrapper {
// =============================================================================

   const int   RBF_POLY_DEG = 1;
   const float MESH_SIZE    = 15.0f;

   // thin plates

#define HERMITE_WITH_X3 1
//#define HERMITE_WITH_THIN_PLATES 1

   // Change type in order to use another phi_function from rbf_phi_funcs.hpp :
#if defined(HERMITE_WITH_X3)
   typedef Rbf_pow3<float> PHI_TYPE;
#elif defined(HERMITE_WITH_THIN_PLATES)
   typedef Rbf_thin_plate<float> PHI_TYPE;
   //typedef Rbf_x_sqrt_x<float> PHI_TYPE;
#endif

}// END RBf_wrapper ============================================================

#endif // HRBF_SETUP_HPP_
