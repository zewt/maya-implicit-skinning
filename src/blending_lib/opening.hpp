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
#ifndef IBL_OPENING_HPP__
#define IBL_OPENING_HPP__

#include "cuda_compiler_interop.hpp"

// =============================================================================
namespace IBL {
// =============================================================================

/**
  @namespace IBL::Opening
  @brief Opening functions defines the boundary between a max
  and the custom profile.

  The opening is defined by the 1D cartesian function 'f[tan_alpha](x)'
  where tan_alpha is a fixed parameter which defines the slope of the opening
  function from the abscissa.
*/
// =============================================================================
namespace Opening {
// =============================================================================

enum Kind_t {
    LINE=0,
    DIAMOND,
    OPEN_TANH,
    CLOSED_H,
    CLOSED_TANH
};

// -----------------------------------------------------------------------------

/// @brief abstract class to evaluate the opening function of a blending operator
class Base {
public:
    /// Evaluate the opening function (f:R->R) at the abscissa x and for an
    /// opening angle tan_alpha
    /// @param x : abscissa x the opening function f
    /// @param tan_alpha : angle from the abscissa which describe the aperture
    /// of the opening function.
    IF_CUDA_DEVICE_HOST
    virtual float f(float x, float tan_alpha) const = 0;
};

/// Factory to create the opening type of "opening_type"
/// @return newly allocated openning
Base* make(Kind_t opening_type);

// -----------------------------------------------------------------------------

/**
 * @brief First piece of the hyperbola openning
 *
 */
class Pan_hf {
public:

    /// @warning Construction in device code is forbidden.
    IF_CUDA_DEVICE_HOST inline
    Pan_hf();

    /// Linear interpolation of _vals[]
    /// @warning call from device is forbidden.
    IF_CUDA_DEVICE_HOST inline
    float linear_fetch(float t) const;

    /// Initialize _vals[] if not already filled.
    /// @warning it only generates the first part of the hyperbola whithout the
    /// connected horyzontal line. note also the hyperbola is scaled by 2
    /// @see _is_init
    static void init_samples();

    /// Resolution for a piece of the profile
    static const int _nb_samples = 1000;
    /// Are _vals[] initialized ?
    static bool _is_init;
    /// samples values of the hyperbola function
    static float _vals[_nb_samples];
};

// -----------------------------------------------------------------------------

/// @brief an hyperbola like function stored discretly and fetch with lerp
///
/// @tparam Data : data needed to compute the hyperbola.
/// Following must be defined :
/// @li Default constructor : IF_CUDA_DEVICE_HOST Data()
/// @li method of signature :
///     IF_CUDA_DEVICE_HOST float linear_fetch(float t) const;
/// @see Pan_hf Discreet_hyperbola
template<class Data>
class Discreet_hyperbola_base : public Base {
public:

    enum Curve_t { CLOSED_HERMITE = 0, ///< Use hermite curve to close the openning
                   CLOSED_TANH,        ///< Use Hyperbola to close the openning
                   OPEN_TANH           ///< Open Opening with hyperbola
                 };

    IF_CUDA_DEVICE_HOST inline
    Discreet_hyperbola_base(Curve_t type);

    /// @warning This function f(x) is scaled by a factor of 2.
    /// its domaine is [0 2] and 1 is suppose to be the isosurface
    IF_CUDA_DEVICE_HOST
    inline float f(float x, float tan_alpha) const;

private:
    IF_CUDA_DEVICE_HOST
    inline float open_pan_hyperbola(float t, float tan_alpha) const;
    IF_CUDA_DEVICE_HOST
    inline float hermite_closure   (float x, float tan_alpha) const;
    IF_CUDA_DEVICE_HOST
    inline float tanh_closure      (float x, float tan_alpha) const;

    Data    _pan_hf; /// First piece of
    Curve_t _type;
};

// -----------------------------------------------------------------------------

/// Shortcut for hyperbola like opening functions when evaluted on CPU
typedef Discreet_hyperbola_base<Pan_hf> Discreet_hyperbola;

// -----------------------------------------------------------------------------

/// @brief a line defined by an affine function
class Line : public Base {
public:
    /// @param tan_alpha : slope of the line
    IF_CUDA_DEVICE_HOST
    inline float f(float x, float tan_alpha) const;
};

// -----------------------------------------------------------------------------

/// Opening function A line between (0., 0.5) then another line connected to it
/// at (0.5, 0.5*tan_alpha) and passing through (_inner, _inner)
class Diamond : public Base {
public:

    IF_CUDA_DEVICE_HOST
    inline Diamond(float inner = 0.7f) : _inner(inner) {}

    /// @warning This function f(x) is scaled by a factor of 2
    IF_CUDA_DEVICE_HOST
    inline float f(float x, float tan_alpha) const;

private:
    const float _inner;
};

}// End Opening ================================================================

}// END IBL ====================================================================

#include "opening.inl"

#endif // IBL_OPENING_HPP__
