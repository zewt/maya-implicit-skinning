#ifndef N_ARY_CONSTANT_INTERFACE_HPP__
#define N_ARY_CONSTANT_INTERFACE_HPP__

/**
 * @file N_ary_constant_interface
 * @brief Interface to access n_ary.hpp constant parameters
*/

// =============================================================================
namespace N_ary {
// =============================================================================

void set_RICCI_N(float v);

void set_wA0A0(float v);
void set_wA0A1(float v);
void set_wA1A1(float v);
void set_wA1A0(float v);
void set_a0   (float v);
void set_w0   (float v);
void set_a1   (float v);
void set_w1   (float v);
void set_gji  (float v);
void set_gij  (float v);

} // END N_ary =================================================================

#endif // N_ARY_CONSTANT_INTERFACE_HPP__
