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
#ifndef CONSTANT_HPP_
#define CONSTANT_HPP_

/** @namespace Constants
    @brief Various float constants stored in a cuda texture.

    The namespace provide an easy interface to read and write float constants
    into cuda texture memory. The value are constants because a CUDA kernel
    can't change their values, unlike the host side of the application. The
    advantage of using cuda texture is the fast access to the data and the
    global scope of the texture.

    Most of the constants are global parameters for the controller of our
    blending function.

    (N.B: the controler usually define the ratio of blending between two types
     of blend operators depending on the input gradient of two surfaces)

    Basic usage :
    @code
    // Initialize data on host memory
    Constants::init();
    // Allocate data on device memory
    Constants::allocate();
    // Update device memory
    Constants::update();

    // You can now access data in texture memory with a kernel :
    float a = Constants::fetch(Constants::B0);

    // or in host global memory
    float a = Constants::get(Constants::B0);

    // If you wish to change a value you can use
    Constants::set(Constants::K3, 0.1f);
    // or which will clamp the value between [0 1]
    Constants::set(Constants::POW0, 0.1f, 0.f, 1.f);

    // you can now access your modified data :
    float a = Constants::get(Constants::POW0);

    // Don't forget to update texture memory before accessing it with a fetch !
    Constants::update();

    // you can now access your modified data in device memory with kernels :
    float a = Constants::fetch(Constants::POW0);

    @endcode

    @warning adding another float constant can be done by just adding its name to
    the enum field, but it must put before the enum NB_CONST, because allocation
    size in host and device array are based on this enum. (Constants has to be
    floats)
*/

// =============================================================================
namespace Constants {
// =============================================================================

    /// ID of the variables
    enum{
        B0, B1, B2, F0, F1, F2, POW0, POW1, K0, K1, K2, K3, K4, K5,
        NB_CONST ///< Must be kept as last enum
    };

    /// Initialize each variable
    void init();

    /// Allocate memory space on the device
    void allocate();

    /// Free memory space on the device
    void free();

    /// Upload the current values of the variables on the device
    /// Device memory must have been allocated with allocate() before
    void update();

    /// Set the value of variable var
    /// Don't forget to update texture memory with update() when changes are
    /// done
    void set(int var, float value);

    /// Increment and clamp variable var
    void incr(int var, float value, float minv, float maxv);

    /// Get the current value of variable var
    float get(int var);

}// END NAMESPACE Constants ====================================================

#endif // CONSTANT_HPP_
