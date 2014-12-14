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
#ifndef IBL_GENERATOR_HPP__
#define IBL_GENERATOR_HPP__

#include "structs.hpp"
#include "tools.hpp"
#include "opening.hpp"

#include <algorithm>
#include <iostream>
#include <cassert>
#include <cmath>

#include "splines.hpp"
#include "vec2_cu.hpp"


// =============================================================================
namespace IBL {
// =============================================================================

class Ctrl_setup;

// TODO: dedicated header for Profile and Profile_polar

/// @namespace IBL::Profile
/// @brief Blending operators profiles defined by 1D functions
// =============================================================================
namespace Profile {
// =============================================================================

/// @brief Abstract class to evaluate 1D function defining the profile
/// of a blending operator
class Base {
public:

    /// Evaluate the profile function at the point of abscisa 'x'
    /// @param x the abscisa
    /// @return profile ordinates
    virtual double f(double x) const = 0;

    /// Evaluate the first derivative of the curve
    /// @param x the abscisa
    /// @return slope of the tangent at the abscisa 'x'
    virtual double df(double x) const = 0;
};

// -----------------------------------------------------------------------------

class Hyperbola : public Base {
public:
    double f(double x) const;
    double df(double x) const;
};

// -----------------------------------------------------------------------------

class Bulge : public Base {
public:
    Bulge(double mag) : _magnitude(mag)
    {  }

    double f(double x) const;
    double df(double x) const;

    /// Magnitude of the bulge in contact. Value range [0.0, 0.9] outside
    /// the operator doesn't behave well
    double _magnitude;
};

// -----------------------------------------------------------------------------

class Ricci_profile : public Base {
public:
    Ricci_profile(double n);

    double f(double x) const;
    double df(double x) const;

    /// N parameter of the ricci operator.
    double _n;
};

// -----------------------------------------------------------------------------

class Spline_profile : public Base{
public:

    /// @param sp : the spline to be converted into a cartesian function y=f(x).
    /// spline should not have multiple images. No checking of spline
    /// consistency is done. Moreover evaluation of f(x) will be clamped at the
    /// first and last control points.
    Spline_profile(const Spline<Vec2_cu, float>& sp);

    /// @warning function is scaled by 2.f
    double f (double x) const;
    double df(double x) const;

    const Spline<Vec2_cu, float>& _spline;
};

}
// END Profile =================================================================


/**
  @namespace IBL::Profile_polar
  @brief describe a 2D curve in polar coordinates to generate a blending
  operator

  Here profile functions defines curves in polar coordinates.
  A curve is defined by f(theta) = dist where theta is an angle
  between [0 pi/2] and dist the distance from the origin to the curve:
  @code

  x x .
  |   /x
  |  /  x
  | /   x
  * ----x

  @endcode
  the x's represents the curve/profile and slashes the line of slope tan(theta)
*/
// =============================================================================
namespace Profile_polar {
// =============================================================================

/// @brief abstract class of polar profile to specialize
class Base {
public:
    /// Evaluate the profile function at the angle tan(alpha)
    /// @param  tan_alpha
    /// @return distance from origin to profile
    virtual float f(float tan_alpha) const = 0;

    virtual IBL::float2 gf(float tan_alpha) const = 0;
};

// -----------------------------------------------------------------------------

/// @brief discreet representation of polar profile
class Discreet : public Base {
public:

    Discreet() :
        _vals(0), _grads(0), _nb_samples(0)
    {    }

    Discreet(float* vals, IBL::float2* grads, int nb_samples) :
        _vals(vals), _grads(grads), _nb_samples(nb_samples)
    {    }

    float f(float tan_alpha) const;
    IBL::float2 gf(float tan_alpha) const;

    void set_data(float* vals, IBL::float2* grads, int nb_samples){
        _vals = vals; _grads = grads; _nb_samples = nb_samples;
    }

    float*       get_vals () const { return _vals;  }
    IBL::float2* get_grads() const { return _grads; }

private:
    /// Access '_vals' values with linear interpolation
    float linear_fetch(float tan_t) const;

    float*       _vals;
    IBL::float2* _grads;
    int          _nb_samples;
};

// -----------------------------------------------------------------------------

/// Profile function for a circle in order to implement barthe's operator
class Circle : public Base {
public:

    float f(float tan_alpha) const;
    IBL::float2 gf(float tan_alpha) const;
};

}
// END Profile =================================================================


/// @param profile profile inside the opening function.
/// @param opening boundary between max and the profile function
/// @param range intervalle you want to precompute g(x, y) operator
/// range being x [0 range] y [0 range]
void gen_custom_operator(const Profile_polar::Base& profile,
                         const Opening::Base& opening,
                         double range,
                         int nb_samples_xy,
                         int nb_samples_alpha,
                         float*& out_values,
                         IBL::float2*& out_gradients);



void gen_polar_profile(Profile_polar::Discreet& output_discreet_curve,
                       const int nb_samples,
                       const Profile::Base& input_curve);


/// @param out_values : '.x' component = values of the controller function 'F'
/// discretize 'nb_samples' times.
/// TODO: comment what's in '.y' for the moment its a mistery. it seems related
/// to the first derivative of F.
///
/// @note F: [-1 1] -> [tan(0) tan(pi/4)]. F input is the dot product between
/// two implicit primitive's gradient. F output is tan( theta ), where theta is
/// the opening angle for the blending operator
void gen_controller(int nb_samples,
                    const Ctrl_setup& shape,
                    IBL::float2*& out_values);

}
// END IBL =====================================================================

#endif // IBL_GENERATOR_HPP__
