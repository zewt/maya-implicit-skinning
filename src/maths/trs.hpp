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
#ifndef TRS_HPP__
#define TRS_HPP__

#include "vec3_cu.hpp"
#include "transfo.hpp"

/**
    @class TRS
    @brief Compact representation of (Translation * Rotation * Scale) matrix

*/
struct TRS {

    /// Default constructor no transformation
    TRS() :
        _axis( Vec3_cu::unit_x() ),
        _angle( 0.f ),
        _scale( Vec3_cu::unit_scale() ),
        _translation( Vec3_cu::zero() )
    {  }

    TRS(const Vec3_cu& trans, const Vec3_cu& axis, float angle, const Vec3_cu& scale) :
        _axis( axis ),
        _angle( angle ),
        _scale( scale ),
        _translation( trans )
    {  }

    /// Build pure rotation
    static TRS rotation(const Vec3_cu& axis, float angle) {
        return TRS(Vec3_cu::zero(), axis, angle, Vec3_cu::unit_scale() );
    }

    /// Build pure translation
    static TRS translation(const Vec3_cu& trans) {
        return TRS(trans, Vec3_cu::unit_x(), 0.f, Vec3_cu::unit_scale() );
    }

    /// Build pure scale
    static TRS scale(const Vec3_cu& scale) {
        return TRS(Vec3_cu::zero(), Vec3_cu::unit_x(), 0.f, scale );
    }

    Transfo to_transfo() {
        return Transfo::translate(_translation) *
               Transfo::rotate(_axis, _angle) *
               Transfo::scale(_scale);
    }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    Vec3_cu _axis;         ///< Axis of rotation
    float   _angle;        ///< angle of rotation
    Vec3_cu _scale;        ///< Scale in (x,y,z) directions
    Vec3_cu _translation;  ///< Vector of translation
};

#endif // TRS_HPP__
