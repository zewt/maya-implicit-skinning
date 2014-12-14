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
#ifndef QUAT_CU_HPP__
#define QUAT_CU_HPP__

#include "transfo.hpp"

/** @class Quaternions implementation to express rotation

  Quaternions are mathematics objects usefull to represent rotation about the
  origin. A quaternion is similar to complex number (a + ib) but in higher
  dimension : q = w + i*x + j*y + k*z where i^2 = j^2 = k^2 = ijk = -1.
  w is the scalar part and (x, y, z) the vector part. The intuition is that
  w represent the amount of rotation about the vector axis which can be
  extracted from the vector part.

  The advantage of dual quaternion is to avoid "gimbal-lock" phenomena and they
  are also slightly faster than their corresponding homogenous matrices.

  You can construct a dual quaternion from a transformation matrix of type
  Transform and rotate a point or a vector with the method 'rotate()'
  it is also possible to compose several rotation by multiplying quaternions
  together with the overload operator (*)

  Quaternion are also used with dual quaternions to represent rotations and
  translations

  @see Dual_quat_cu
*/
// =============================================================================
class Quat_cu{
// =============================================================================
    public:

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    /// Default constructor : build a zero rotation.
    IF_CUDA_DEVICE_HOST
    Quat_cu()
    {
        coeff[0] = 1.f;
        coeff[1] = 0.f; coeff[2] = 0.f; coeff[3] = 0.f;
    }

    /// Copy constructor
    IF_CUDA_DEVICE_HOST
    Quat_cu(const Quat_cu& q){
        coeff[0] = q.w();
        coeff[1] = q.i(); coeff[2] = q.j(); coeff[3] = q.k();
    }

    /// directly fill the quaternion
    IF_CUDA_DEVICE_HOST
    Quat_cu(float w, float i, float j, float k){
        coeff[0] = w;
        coeff[1] = i; coeff[2] = j; coeff[3] = k;
    }

    /// directly fill the quaternion vector and scalar part
    IF_CUDA_DEVICE_HOST
    Quat_cu(float w, const Vec3_cu& v){
        coeff[0] = w;
        coeff[1] = v.x; coeff[2] = v.y; coeff[3] = v.z;
    }

    /// Construct the quaternion from the transformation matrix 't'
    IF_CUDA_DEVICE_HOST
    Quat_cu(const Transfo& t)
    {
        // Compute trace of matrix 't'
        float T = 1 + t.m[0] + t.m[5] + t.m[10];

        float S, X, Y, Z, W;

        if ( T > 0.00000001f ) // to avoid large distortions!
        {
            S = sqrt(T) * 2.f;
            X = ( t.m[6] - t.m[9] ) / S;
            Y = ( t.m[8] - t.m[2] ) / S;
            Z = ( t.m[1] - t.m[4] ) / S;
            W = 0.25f * S;
        }
        else
        {
            if ( t.m[0] > t.m[5] && t.m[0] > t.m[10] )
            {
                // Column 0 :
                S  = sqrt( 1.0f + t.m[0] - t.m[5] - t.m[10] ) * 2.f;
                X = 0.25f * S;
                Y = (t.m[1] + t.m[4] ) / S;
                Z = (t.m[8] + t.m[2] ) / S;
                W = (t.m[6] - t.m[9] ) / S;
            }
            else if ( t.m[5] > t.m[10] )
            {
                // Column 1 :
                S  = sqrt( 1.0f + t.m[5] - t.m[0] - t.m[10] ) * 2.f;
                X = (t.m[1] + t.m[4] ) / S;
                Y = 0.25f * S;
                Z = (t.m[6] + t.m[9] ) / S;
                W = (t.m[8] - t.m[2] ) / S;
            }
            else
            {   // Column 2 :
                S  = sqrt( 1.0f + t.m[10] - t.m[0] - t.m[5] ) * 2.f;
                X = (t.m[8] + t.m[2] ) / S;
                Y = (t.m[6] + t.m[9] ) / S;
                Z = 0.25f * S;
                W = (t.m[1] - t.m[4] ) / S;
            }
        }

        coeff[0] = W; coeff[1] = -X; coeff[2] = -Y; coeff[3] = -Z;
    }


    /// Construct the quaternion from the a rotation axis 'axis' and the angle
    /// 'angle'
    IF_CUDA_DEVICE_HOST
    Quat_cu(const Vec3_cu& axis, float angle)
    {
        Vec3_cu vec_axis = axis.normalized();
        float sin_a = sin( angle * 0.5f );
        float cos_a = cos( angle * 0.5f );
        coeff[0]    = cos_a;
        coeff[1]    = vec_axis.x * sin_a;
        coeff[2]    = vec_axis.y * sin_a;
        coeff[3]    = vec_axis.z * sin_a;
        // It is necessary to normalize the quaternion in case any values are
        // very close to zero.
        normalize();
    }

    // -------------------------------------------------------------------------
    /// @name Methods
    // -------------------------------------------------------------------------

    /// The conjugate of a quaternion is the inverse rotation
    /// (when the quaternion is normalized
    IF_CUDA_DEVICE_HOST
    Quat_cu conjugate() const
    {
        return Quat_cu( coeff[0], -coeff[1],
                       -coeff[2], -coeff[3]);
    }

    // TODO: Construct the quaternion from the rotation axis 'vec' and the
    // angle 'angle'
    // Quat_cu(const Vec3_cu& vec, float angle)

    /// Do the rotation of vector 'v' with the quaternion
    IF_CUDA_DEVICE_HOST
    Vec3_cu rotate(const Vec3_cu& v) const
    {

        // The conventionnal way to rotate a vector
        /*
        Quat_cu tmp = *this;
        tmp.normalize();
        // Compute the quaternion inverse with
        Quat_cu inv = tmp.conjugate();
        // Compute q * v * inv; in order to rotate the vector v
        // to do so v must be expressed as the quaternion q(0, v.x, v.y, v.z)
        return (Vec3_cu)(*this * Quat_cu(0, v) * inv);
        */

        // An optimized way to compute rotation
        Vec3_cu q_vec = get_vec_part();
        return v + (q_vec*2.f).cross( q_vec.cross(v) + v*coeff[0] );
    }

    /// Do the rotation of point 'p' with the quaternion
    IF_CUDA_DEVICE_HOST
    Point_cu rotate(const Point_cu& p) const
    {
        Vec3_cu v = rotate((Vec3_cu)p);
        return Point_cu(v.x, v.y, v.z);
    }

    /// Convert the quaternion to a rotation matrix
    /// @warning don't forget to normalize it before conversion
    IF_CUDA_DEVICE_HOST
    Mat3_cu to_matrix3()
    {
        float W = coeff[0], X = -coeff[1], Y = -coeff[2], Z = -coeff[3];
        float xx = X * X, xy = X * Y, xz = X * Z, xw = X * W;
        float yy = Y * Y, yz = Y * Z, yw = Y * W, zz = Z * Z;
        float zw = Z * W;
        Mat3_cu mat = Mat3_cu(
                    1.f - 2.f * (yy + zz),      2.f * (xy + zw),       2.f * (xz - yw),
                          2.f * (xy - zw),1.f - 2.f * (xx + zz),       2.f * (yz - xw),
                          2.f * (xz + yw),      2.f * (yz + xw), 1.f - 2.f * (xx + yy)
                    );
        return mat;
    }

    IF_CUDA_DEVICE_HOST
    Vec3_cu get_vec_part() const
    {
        return Vec3_cu(coeff[1], coeff[2], coeff[3]);
    }

    IF_CUDA_DEVICE_HOST
    float norm() const
    {
        return sqrt(coeff[0]*coeff[0] +
                    coeff[1]*coeff[1] +
                    coeff[2]*coeff[2] +
                    coeff[3]*coeff[3]);
    }

    IF_CUDA_DEVICE_HOST
    float normalize()
    {
        float n = norm();
        coeff[0] /= n;
        coeff[1] /= n;
        coeff[2] /= n;
        coeff[3] /= n;
        return n;
    }

    IF_CUDA_DEVICE_HOST
    float dot(const Quat_cu& q){
        return w() * q.w() + i() * q.i() + j() * q.j() + k() * q.k();
    }

    IF_CUDA_DEVICE_HOST float w() const { return coeff[0]; }
    IF_CUDA_DEVICE_HOST float i() const { return coeff[1]; }
    IF_CUDA_DEVICE_HOST float j() const { return coeff[2]; }
    IF_CUDA_DEVICE_HOST float k() const { return coeff[3]; }

    // -------------------------------------------------------------------------
    /// @name Operators
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    Quat_cu operator/ (float scalar) const
    {
        Quat_cu q = *this;
        q.coeff[0] /= scalar;
        q.coeff[1] /= scalar;
        q.coeff[2] /= scalar;
        q.coeff[3] /= scalar;
        return q;
    }

    IF_CUDA_DEVICE_HOST
    Quat_cu operator/= (float scalar){
        coeff[0] /= scalar;
        coeff[1] /= scalar;
        coeff[2] /= scalar;
        coeff[3] /= scalar;
        return *this;
    }

    IF_CUDA_DEVICE_HOST
    Quat_cu operator* (const Quat_cu& q) const
    {
         return Quat_cu(
         coeff[0]*q.coeff[0] - coeff[1]*q.coeff[1] - coeff[2]*q.coeff[2] - coeff[3]*q.coeff[3],
         coeff[0]*q.coeff[1] + coeff[1]*q.coeff[0] + coeff[2]*q.coeff[3] - coeff[3]*q.coeff[2],
         coeff[0]*q.coeff[2] + coeff[2]*q.coeff[0] + coeff[3]*q.coeff[1] - coeff[1]*q.coeff[3],
         coeff[0]*q.coeff[3] + coeff[3]*q.coeff[0] + coeff[1]*q.coeff[2] - coeff[2]*q.coeff[1]);
    }

    IF_CUDA_DEVICE_HOST
    Quat_cu operator* (float scalar) const
    {
        return Quat_cu(coeff[0] * scalar,
                       coeff[1] * scalar,
                       coeff[2] * scalar,
                       coeff[3] * scalar);
    }

    IF_CUDA_DEVICE_HOST
    Quat_cu operator+ (const Quat_cu& q) const
    {
         return Quat_cu(coeff[0] + q.coeff[0],
                        coeff[1] + q.coeff[1],
                        coeff[2] + q.coeff[2],
                        coeff[3] + q.coeff[3]);
    }

    /// Get vector part
    IF_CUDA_DEVICE_HOST
    operator Vec3_cu () const{
        return Vec3_cu(coeff[1], coeff[2], coeff[3]);
    }

    /// Get scalar part
    IF_CUDA_DEVICE_HOST
    operator float () const{
        return coeff[0];
    }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    /// coeff[0], coeff[1], coeff[2], coeff[3] respectively
    /// w, i, j, k coefficients or W, X, Y, Z as noted in the F.A.Q
    float coeff[4];

};
// =============================================================================

#endif // QUAT_CU_HPP__
