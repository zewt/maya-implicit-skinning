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
#ifndef CPU_TRANSFO_HPP
#define CPU_TRANSFO_HPP

// This is nothing but the Vec3_cu, etc. classes copied without CUDA support.  All of this
// nonsense is needed only because the CUDA and Maya headers are incompatible.  This is only
// used during mesh loading, and we switch to *_cu as soon as possible.
namespace Loader {
    struct Point;
    struct Vec3 {

        float x, y, z;

        Vec3() { x = 0.f; y = 0.f; z = 0.f; }

        Vec3(float x_, float y_, float z_) { x = x_; y = y_; z = z_; }

        static inline Vec3 unit_x(){
            return Vec3(1.f, 0.f, 0.f);
        }

        static inline Vec3 unit_y(){
            return Vec3(0.f, 1.f, 0.f);
        }

        static inline Vec3 unit_z(){
            return Vec3(0.f, 0.f, 1.f);
        }

        static inline Vec3 zero() {
            return Vec3(0.f, 0.f, 0.f);
        }

        static inline Vec3 unit_scale(){
            return Vec3(1.f, 1.f, 1.f);
        }

        inline void set(float x_, float y_, float z_) {
            x = x_; y = y_; z = z_;
        }

        #ifdef __CUDACC__
        __device__ __host__
        float4 to_float4() const{
            return make_float4(x, y, z, 0.f);
        }
        #endif

        static Vec3 random(float r){
            float r2 = 2.f * r;
            float x_ = rand() * 1.f /RAND_MAX;
            float y_ = rand() * 1.f /RAND_MAX;
            float z_ = rand() * 1.f /RAND_MAX;
            return Vec3(x_ * r2 - r, y_ * r2 - r, z_ * r2 - r);
        }

        // -------------------------------------------------------------------------
        /// @name Overload operators
        // -------------------------------------------------------------------------

        /// addition
        Vec3 operator+(const Vec3 &v_) const {
            return Vec3(x+v_.x, y+v_.y, z+v_.z);
        }

        Vec3 operator*(const Vec3 &v_) const {
            return Vec3(x*v_.x, y*v_.y, z*v_.z);
        }

        Vec3& operator+= (const Vec3 &v_) {
            x += v_.x;
            y += v_.y;
            z += v_.z;
            return *this;
        }

        bool operator!= (const Vec3 &v_) const {
            return (x != v_.x) |  (y != v_.y) | (z != v_.z);
        }

        Vec3 operator+(float f_) const {
            return Vec3(x+f_, y+f_, z+f_);
        }

        Vec3& operator+= (float f_) {
            x += f_;
            y += f_;
            z += f_;
            return *this;
        }

        /// substraction
        Vec3 operator-(const Vec3 &v_) const {
            return Vec3(x-v_.x, y-v_.y, z-v_.z);
        }

        /// opposite vector
        Vec3 operator-() const {
            return Vec3(-x, -y, -z);
        }

        /// scalar multiplication
        Vec3 operator*(const float d_) const {
            return Vec3(x*d_, y*d_, z*d_);
        }

        Vec3 operator/(const float d_) const {
            return Vec3(x/d_, y/d_, z/d_);
        }

        Vec3& operator/=(const float d_) {
            x /= d_;
            y /= d_;
            z /= d_;
            return *this;
        }

        Vec3& operator*=(const float d_) {
            x *= d_;
            y *= d_;
            z *= d_;
            return *this;
        }

        Vec3& operator*=(const Vec3& d_) {
            x *= d_.x;
            y *= d_.y;
            z *= d_.z;
            return *this;
        }

        // -------------------------------------------------------------------------
        /// @name Operators on vector
        // -------------------------------------------------------------------------

        /// product of all components
        float product() const { return x*y*z; }

        /// sum of all components
        float sum() const { return x+y+z; }

        /// semi dot product (component wise multiplication)
        Vec3 mult(const Vec3& v) const {
            return Vec3(x*v.x, y*v.y, z*v.z);
        }

        /// component wise division
        Vec3 div(const Vec3& v) const {
            return Vec3(x/v.x, y/v.y, z/v.z);
        }

        /// cross product
        Vec3 cross(const Vec3& v_) const {
            return Vec3(y*v_.z-z*v_.y, z*v_.x-x*v_.z, x*v_.y-y*v_.x);
        }


        /// dot product
        float dot(const Vec3& v_) const {
            return x*v_.x+y*v_.y+z*v_.z;
        }

        /// Compute the cotangente (1./tan) between 'this' and v_
        float cotan(const Vec3& v_) const {
            // cot(alpha ) = dot(v1, v2) / ||cross(v1, v2)||
            // = ||v1||*||v2||*cos( angle(v1, v2) ) / ||v1||*||v2|| * sin( angle(v1, v2) )
            // = cos( angle(v1, v2) ) / sin( angle(v1, v2) )
            // = 1 / tan( angle(v1, v2) )
            // = cot( angle(v1, v2) ) = cot( alpha )
            return (this->dot(v_)) / (this->cross(v_)).norm();
        }

        /// absolute value of the dot product
        float abs_dot(const Vec3& v_) const {
            return fabsf(x * v_.x + y * v_.y + z * v_.z);
        }

        /// norm squared
        float norm_squared() const {
            return dot(*this);
        }

        /// normalization
        Vec3 normalized() const {
            return (*this) * (1.f/sqrtf(norm_squared()));
        }

        /// normalization
        float normalize() {
            float l = sqrtf(norm_squared());
            float f = 1.f / l;
            x *= f;
            y *= f;
            z *= f;
            return l;
        }

        /// normalization
        float safe_normalize() {
            float l = sqrtf(norm_squared());
            if(l > 1e-10f){
                float f = 1.f / l;
                x *= f;
                y *= f;
                z *= f;
                return l;
            } else {
                x = 1.f;
                y = 0.f;
                z = 0.f;
                return 0.f;
            }
        }

        /// norm
        float norm() const {
            return sqrtf(norm_squared());
        }

        /// value of the min coordinate
        float get_min() const {
            return fminf(fminf(x,y),z);
        }

        /// value of the max coordinate
        float get_max() const {
            return fmaxf(fmaxf(x,y),z);
        }

        /// clamp each vector values
        Vec3 clamp(float min_v, float max_v) const {
            return Vec3( fminf( fmaxf(x, min_v), max_v),
                            fminf( fmaxf(y, min_v), max_v),
                            fminf( fmaxf(z, min_v), max_v));
        }

        /// rotate of 0 step to the left (present for symmetry)
        Vec3 perm_x() const {
            return Vec3(x, y, z);
        }

        /// rotate of 1 step to the left (so that y is the first coordinate)
        Vec3 perm_y() const {
            return Vec3(y, z, x);
        }

        /// rotate of 2 steps to the left (so that z is the first coordinate)
        Vec3 perm_z() const {
            return Vec3(z, x, y);
        }

        void coordinate_system (Vec3& v1_, Vec3& v2_) const {
            //for numerical stability, and seen that z will
            //always be present, take the greatest component between
            //x and y.
            if( fabsf(x) > fabsf(y) ) {
                float inv_len = 1.f / sqrtf(x * x + z * z);
                Vec3 tmp(-z * inv_len, 0.f, x * inv_len);
                v1_ = tmp;
            } else {
                float inv_len = 1.f / sqrtf (y * y + z * z);
                Vec3 tmp(0.f, z * inv_len, -y * inv_len);
                v1_ = tmp;
            }
            v2_ = (*this).cross (v1_);
        }

        /// Get a random orthogonal vector
        Vec3 get_ortho() const
        {
            Vec3 ortho = this->cross(Vec3(1.f, 0.f, 0.f));

            if (ortho.norm_squared() < 1e-06f * 1e-06f)
                ortho = this->cross( Vec3(0.f, 1.f, 0.f) );

            return ortho;
        }

        /// @return the vector to_project projected on the plane defined by the
        /// normal '*this'
        /// @warning don't forget to normalize the vector before calling
        /// proj_on_plane() !
        Vec3 proj_on_plane(const Vec3& to_project) const
        {
            return ( (*this).cross(to_project) ).cross( (*this) );
        }

        /// @return the point to_project projected on the plane defined by the
        /// normal '*this' and passing through pos_plane
        /// @warning don't forget to normalize the vector before calling
        /// proj_on_plane() !
        /// @note implemented in Point.h because of cross definitions
        inline Point proj_on_plane(const Point& pos_plane,
                                      const Point& to_project) const;

        /// @note implemented in Point.h because of cross definitions
        inline Point to_point() const;

        inline const float& operator[](int i) const{
            switch(i){
            case 0: return x;
            case 1: return y;
            }
            return z;
        }

        inline float& operator[](int i){
            switch(i){
            case 0: return x;
            case 1: return y;
            }
            return z;
        }

        void print(){
            printf("%f, %f, %f\n", x, y, z);
        }

    };

    struct Point {

        float x, y, z;

        Point(){
            x = 0.f; y = 0.f; z = 0.f;
        }

        Point(float a, float b, float c){
            x = a; y = b; z = c;
        }

        inline void set(float x_, float y_, float z_) {
            x = x_; y = y_; z = z_;
        }

        static Point random(float r){
            float r2 = 2.f * r;
            float x_ = rand() * 1.f /RAND_MAX;
            float y_ = rand() * 1.f /RAND_MAX;
            float z_ = rand() * 1.f /RAND_MAX;
            return Point(x_ * r2 - r, y_ * r2 - r, z_ * r2 - r);
        }

        /// displacement
        Point operator+(const Vec3 &v_) const {
            return Point(x+v_.x, y+v_.y, z+v_.z);
        }

        /// displacement
        Point operator-(const Vec3 &v_) const {
            return Point(x-v_.x, y-v_.y, z-v_.z);
        }

        /// difference
        Vec3 operator-(const Point &p_) const {
            return Vec3(x-p_.x, y-p_.y, z-p_.z);
        }

        /// opposite point
        Point operator-() const {
            return Point(-x, -y, -z);
        }

        Point operator/(float s) const {
            return Point(x/s, y/s, z/s);
        }

        /// squared distance to another point
        float distance_squared (const Point& p_) const {
            return (p_ - *this).norm_squared();
        }


        /// value of the min coordinate
        float get_min() const {
            return fminf(fminf(x,y),z);
        }

        /// value of the max coordinate
        float get_max() const {
            return fmaxf(fmaxf(x,y),z);
        }

        // TODO: supr this and use explicit function like to_vector()
        operator Vec3() const {
            return Vec3(x, y, z);
        }

        Vec3 to_vector() const {
            return Vec3(x, y, z);
        }

        inline Point operator+(const Point& p) const {
            return Point(x + p.x, y + p.y, z + p.z);
        }

        inline Point operator*(float f) const {
            return Point(x * f, y * f, z * f);
        }

        inline const float& operator[](int i) const{
            switch(i){
            case 0: return x;
            case 1: return y;
            }
            return z;
        }

        inline float& operator[](int i){
            switch(i){
            case 0: return x;
            case 1: return y;
            }
            return z;
        }

        inline Point perm_x() const{
            return Point(x, y, z);
        }

        inline Point perm_y() const{
            return Point(y, z, x);
        }

        inline Point perm_z() const{
            return Point(z, x, y);
        }

        inline void print() const {
            printf("(%f,%f,%f) ", x, y, z);
        }
    };

    inline Point Vec3::proj_on_plane(const Point& pos_plane,
                                           const Point& to_project) const
    {
        return to_project + (*this) * (pos_plane - to_project).dot( (*this) );
    }

    inline Point Vec3::to_point() const{
        return Point(x, y, z);
    }


        struct Mat3 {

        float a, b, c; ///< first row
        float d, e, f; ///< second row
        float g, h ,i; ///< third row

        inline Mat3() {   }

        inline Mat3(float a_, float b_, float c_,
                float d_, float e_, float f_,
                float g_, float h_, float i_)
        {
            a = a_; b = b_; c = c_;
            d = d_; e = e_; f = f_;
            g = g_; h = h_; i = i_;
        }

        inline Mat3(const Vec3& x,
                const Vec3& y,
                const Vec3& z)
        {
            a = x.x; b = y.x; c = z.x;
            d = x.y; e = y.y; f = z.y;
            g = x.z; h = y.z; i = z.z;
        }

        Vec3 operator*(const Vec3& v) const
        {
            float x = v.x * a + v.y * b + v.z * c;
            float y = v.x * d + v.y * e + v.z * f;
            float z = v.x * g + v.y * h + v.z * i;
            return Vec3(x, y, z);
        }

        inline Mat3 operator*(const Mat3& m) const
        {
            return Mat3(a * m.a + b * m.d + c * m.g,
                           a * m.b + b * m.e + c * m.h,
                           a * m.c + b * m.f + c * m.i,
                           d * m.a + e * m.d + f * m.g,
                           d * m.b + e * m.e + f * m.h,
                           d * m.c + e * m.f + f * m.i,
                           g * m.a + h * m.d + i * m.g,
                           g * m.b + h * m.e + i * m.h,
                           g * m.c + h * m.f + i * m.i);
        }

        inline Mat3 operator*(float x) const
        {
            return Mat3(a * x, b * x, c * x,
                           d * x, e * x, f * x,
                           g * x, h * x, i * x);
        }


        inline Mat3 operator+(const Mat3& m) const
        {
            return Mat3(a + m.a, b + m.b, c + m.c,
                           d + m.d, e + m.e, f + m.f,
                           g + m.g, h + m.h, i + m.i);
        }


        inline Mat3 operator-(const Mat3& m) const
        {
            return Mat3(a - m.a, b - m.b, c - m.c,
                           d - m.d, e - m.e, f - m.f,
                           g - m.g, h - m.h, i - m.i);
        }

        inline float det() const
        {
            return a * ( e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        }

        /// @return the matrix with normalized x, y, z column vectors
        inline Mat3 normalized() const {
            return Mat3(x().normalized(), y().normalized(), z().normalized());
        }

        inline Mat3 inverse() const
        {
            float c0 = e * i - f * h;
            float c1 = f * g - d * i;
            float c2 = d * h - e * g;
            float idet = 1.f / (a * c0 + b * c1 + c * c2);
            return Mat3(c0 , c * h - b * i, b * f - c * e,
                           c1 , a * i - c * g, c * d - a * f,
                           c2 , b * g - a * h, a * e - b * d) * idet;
        }

        inline Mat3 transpose() const
        {
            return Mat3(a, d, g, b, e, h, c, f, i);
        }

        inline void set_abs()
        {
            a = fabs(a); b = fabs(b); c = fabs(c);
            d = fabs(d); e = fabs(e); f = fabs(f);
            g = fabs(g); h = fabs(h); i = fabs(i);
        }


        inline float max_elt() const
        {
            return fmaxf(i, fmaxf(fmaxf(fmaxf(a,b),fmaxf(c,d)),
                                  fmaxf(fmaxf(e,f),fmaxf(g,h))));
        }

        inline float min_elt() const
        {
            return fminf(i, fminf(fminf(fminf(a,b),fminf(c,d)),
                                  fminf(fminf(e,f),fminf(g,h))));
        }

        Mat3 get_ortho() const
        {
            Mat3 h0 = (*this);
            Mat3 h1 = h0;
            h1.set_abs();
            float eps =(1.f +  h1.min_elt()) * 1e-5f;
            for(int i = 0; i < 500/* to avoid infinite loop */; i++){
                h0 = (h0 + (h0.inverse()).transpose()) * 0.5f;
                h1 = h1 - h0;
                h1.set_abs();
                if(h1.max_elt() <= eps)
                    break;
                h1 = h0;
            }
            return h0;
        }

        float get_rotation_axis_angle(Vec3& axis) const
        {
            axis.x = h - f + 1e-5f;
            axis.y = c - g;
            axis.z = d - b;
            float sin_angle = axis.safe_normalize();
            float cos_angle = a + e + i - 1.f;
            return atan2(sin_angle, cos_angle);
        }

        inline Vec3 x() const { return Vec3(a, d, g); }
        inline Vec3 y() const { return Vec3(b, e, h); }
        inline Vec3 z() const { return Vec3(c, f, i); }

        // =========================================================================
        /// @name Static constructors
        // =========================================================================

        static inline Mat3 identity()
        {
            return Mat3(1.f, 0.f, 0.f,
                           0.f, 1.f, 0.f,
                           0.f, 0.f, 1.f);

        }

        /// @return the rotation matrix given 'axis' and 'angle' in radian
        static inline Mat3 rotate(const Vec3& axis, float angle)
        {
            Vec3 n = axis;
            n.normalize();
            float cp = cosf(angle);
            float sp = sinf(angle);
            float acp = 1.f - cp;
            float nxz = n.x * n.z;
            float nxy = n.x * n.y;
            float nyz = n.y * n.z;
            return Mat3(cp + acp * n.x * n.x,
                           acp * nxy - sp * n.z,
                           acp * nxz + sp * n.y,

                           acp * nxy + sp * n.z,
                           cp + acp * n.y * n.y,
                           acp * nyz - sp * n.x,

                           acp * nxz - sp * n.y,
                           acp * nyz + sp * n.x,
                           cp + acp * n.z * n.z);
        }

        /// @return a orthogonal/normalized frame with its x axis aligned to v
        static inline Mat3 coordinate_system(const Vec3& v)
        {
            Vec3 fx, fy, fz;
            fx = v.normalized();
            fx.coordinate_system(fy, fz);
            return Mat3(fx, fy, fz);
        }


    };


    
    /**
      @name CpuTransfo
      @brief Handling geometric transformations with a 4x4 matrix


      @see Mat3 Vec3
    */
    struct CpuTransfo {

        /// Linear matrix storage with <b> rows first (i.e row major) </b>
        /// Using this with OpenGL can be done by transposing first:
        /// @code
        ///     CpuTransfo tr;
        ///     // OpenGL is column major !
        ///     glMultMatrixf( (GLfloat)(tr.transpose().m) );
        /// @endcode
        float m[16];

        // -------------------------------------------------------------------------
        /// @name Constructors
        // -------------------------------------------------------------------------

        inline CpuTransfo() {}

        inline CpuTransfo(float a00, float a01, float a02, float a03,
                       float a10, float a11, float a12, float a13,
                       float a20, float a21, float a22, float a23,
                       float a30, float a31, float a32, float a33)
        {
            m[ 0] = a00; m[ 1] = a01; m[ 2] = a02; m[ 3] = a03;
            m[ 4] = a10; m[ 5] = a11; m[ 6] = a12; m[ 7] = a13;
            m[ 8] = a20; m[ 9] = a21; m[10] = a22; m[11] = a23;
            m[12] = a30; m[13] = a31; m[14] = a32; m[15] = a33;
        }

        inline CpuTransfo(const Mat3& x){
            m[ 0] = x.a; m[ 1] = x.b; m[ 2] = x.c; m[ 3] = 0.f;
            m[ 4] = x.d; m[ 5] = x.e; m[ 6] = x.f; m[ 7] = 0.f;
            m[ 8] = x.g; m[ 9] = x.h; m[10] = x.i; m[11] = 0.f;
            m[12] = 0.f; m[13] = 0.f; m[14] = 0.f; m[15] = 1.f;
        }

        inline CpuTransfo(const Mat3& x, const Vec3& v){
            m[ 0] = x.a; m[ 1] = x.b; m[ 2] = x.c; m[ 3] = v.x;
            m[ 4] = x.d; m[ 5] = x.e; m[ 6] = x.f; m[ 7] = v.y;
            m[ 8] = x.g; m[ 9] = x.h; m[10] = x.i; m[11] = v.z;
            m[12] = 0.f; m[13] = 0.f; m[14] = 0.f; m[15] = 1.f;
        }

        inline CpuTransfo(const Vec3& v){
            m[ 0] = 1.f; m[ 1] = 0.f; m[ 2] = 0.f; m[ 3] = v.x;
            m[ 4] = 0.f; m[ 5] = 1.f; m[ 6] = 0.f; m[ 7] = v.y;
            m[ 8] = 0.f; m[ 9] = 0.f; m[10] = 1.f; m[11] = v.z;
            m[12] = 0.f; m[13] = 0.f; m[14] = 0.f; m[15] = 1.f;
        }

        // -------------------------------------------------------------------------
        /// @name Setters
        // -------------------------------------------------------------------------

        inline void set_x(const Vec3& x){
            m[0] = x.x; m[4] = x.y; m[8] = x.z;
        }

        inline void set_y(const Vec3& y){
            m[1] = y.x; m[5] = y.y; m[9] = y.z;
        }

        inline void set_z(const Vec3& z){
            m[2] = z.x; m[6] = z.y; m[10] = z.z;
        }

        inline Vec3 x() const{
            return Vec3( m[0], m[4], m[8] );
        }

        inline Vec3 y() const{
            return Vec3( m[1], m[5], m[9] );
        }

        inline Vec3 z() const{
            return Vec3( m[2], m[6], m[10] );
        }

        inline void set_translation(const Vec3& tr){
            m[3] = tr.x; m[7] = tr.y; m[11] = tr.z;
        }

        inline void set_org(const Vec3& tr){
            m[3] = tr.x; m[7] = tr.y; m[11] = tr.z;
        }

        inline void set_translation(const CpuTransfo& tr){
            const Vec3 trans = tr.get_translation();
            m[3] = trans.x; m[7] = trans.y; m[11] = trans.z;
        }

        inline void set_mat3(const Mat3& x){
            m[ 0] = x.a; m[ 1] = x.b; m[ 2] = x.c;
            m[ 4] = x.d; m[ 5] = x.e; m[ 6] = x.f;
            m[ 8] = x.g; m[ 9] = x.h; m[10] = x.i;
        }

        // -------------------------------------------------------------------------
        /// @name Operators
        /// @note A special attention has to be made regarding the multiplication
        /// operators. The operators are overloaded differently wether you use
        /// vectors or points. Vectors does not need to be multiplied against the
        /// matrix translation part, whereas points does. This is because the
        /// homogenous part is not represented. Yet when projecting a point you will
        /// need the fourth component. In this case projection can't be done through
        /// matrix multiplication use instead the method 'project()'
        // -------------------------------------------------------------------------

        inline Vec3 operator*(const Vec3& v) const {
            return Vec3(
                        m[0] * v.x + m[1] * v.y + m[ 2] * v.z,
                        m[4] * v.x + m[5] * v.y + m[ 6] * v.z,
                        m[8] * v.x + m[9] * v.y + m[10] * v.z);
        }

        /// @warning this ignores the homogenous coordinates use project() to
        /// consider the fourth component and do the perspective division for
        /// projection matrices
        inline Point operator*(const Point& v) const {
            return Point(
                        m[0] * v.x + m[1] * v.y + m[ 2] * v.z + m[ 3],
                        m[4] * v.x + m[5] * v.y + m[ 6] * v.z + m[ 7],
                        m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]);
        }

        /// Multiply 'v' by the matrix and do the perspective division
        inline Point project(const Point& v) const {
            Point tmp =  Point(
                        m[0] * v.x + m[1] * v.y + m[ 2] * v.z + m[ 3],
                        m[4] * v.x + m[5] * v.y + m[ 6] * v.z + m[ 7],
                        m[8] * v.x + m[9] * v.y + m[10] * v.z + m[11]);

            return tmp / (m[12] * v.x + m[13] * v.y + m[14] * v.z + m[15]);
        }


        inline CpuTransfo operator*(const CpuTransfo& t) const {
            CpuTransfo res;
            for(int i = 0; i < 4; i++){
                int j = i*4;
                res[j+0] = m[j] * t.m[0] + m[j+1] * t.m[4] + m[j+2] * t.m[ 8] + m[j+3] * t.m[12];
                res[j+1] = m[j] * t.m[1] + m[j+1] * t.m[5] + m[j+2] * t.m[ 9] + m[j+3] * t.m[13];
                res[j+2] = m[j] * t.m[2] + m[j+1] * t.m[6] + m[j+2] * t.m[10] + m[j+3] * t.m[14];
                res[j+3] = m[j] * t.m[3] + m[j+1] * t.m[7] + m[j+2] * t.m[11] + m[j+3] * t.m[15];
            }
            return res;
        }

        inline CpuTransfo& operator*=(const CpuTransfo& t){
            (*this) = (*this) * t;
            return (*this);
        }

        inline Vec3 vec_prod(const Vec3& v) const {
            return Vec3(
                        m[0] * v.x + m[1] * v.y + m[ 2] * v.z,
                        m[4] * v.x + m[5] * v.y + m[ 6] * v.z,
                        m[8] * v.x + m[9] * v.y + m[10] * v.z);
        }

        inline CpuTransfo operator*(float x) const {
            CpuTransfo res;
            for(int i = 0; i < 16; i++){
                res[i] = m[i] * x;
            }
            return res;
        }

        inline CpuTransfo operator+(const CpuTransfo& t) const{
            CpuTransfo res;
            for(int i = 0; i < 16; i++){
                res[i] = m[i] + t.m[i];
            }
            return res;
        }

        inline CpuTransfo& operator+=(const CpuTransfo& t){
            (*this) = (*this) + t;
            return (*this);
        }

        inline CpuTransfo operator-(const CpuTransfo& t) const{
            CpuTransfo res;
            for(int i = 0; i < 16; i++){
                res[i] = m[i] - t.m[i];
            }
            return res;
        }

        inline CpuTransfo& operator-=(const CpuTransfo& t){
            (*this) = (*this) - t;
            return (*this);
        }

        inline float& operator[](int i) {
            return m[i];
        }

        inline const float& operator[](int i) const {
            return m[i];
        }

        // -------------------------------------------------------------------------
        /// @name Getters
        // -------------------------------------------------------------------------


        inline CpuTransfo transpose() const {
            return CpuTransfo(m[0], m[4], m[ 8], m[12],
                           m[1], m[5], m[ 9], m[13],
                           m[2], m[6], m[10], m[14],
                           m[3], m[7], m[11], m[15]);
        }

        /// Fast inversion of the Transformation matrix. To accelerate computation
        /// we consider that the matrix only represents affine Transformations
        /// such as rotation, scaling, translation, shear...
        /// @warning don't use this procedure to invert a projection matrix. Instead
        /// use full_invert().
        inline CpuTransfo fast_invert() const {
            Mat3 a(m[0], m[1], m[ 2],
                      m[4], m[5], m[ 6],
                      m[8], m[9], m[10]);

            Vec3 b(m[3], m[7], m[11]);
            Mat3 x = a.inverse();
            Vec3 y = x * b;
            return CpuTransfo(x.a, x.b, x.c, -y.x,
                           x.d, x.e, x.f, -y.y,
                           x.g, x.h, x.i, -y.z,
                           0.f, 0.f, 0.f,  1.f);
        }

        float MINOR(const CpuTransfo& m, const int r0, const int r1, const int r2, const int c0, const int c1, const int c2) const {
            return m[4*r0+c0] * (m[4*r1+c1] * m[4*r2+c2] - m[4*r2+c1] * m[4*r1+c2]) -
                   m[4*r0+c1] * (m[4*r1+c0] * m[4*r2+c2] - m[4*r2+c0] * m[4*r1+c2]) +
                   m[4*r0+c2] * (m[4*r1+c0] * m[4*r2+c1] - m[4*r2+c0] * m[4*r1+c1]);
        }

        CpuTransfo adjoint() const {
            return CpuTransfo( MINOR(*this,1,2,3,1,2,3), -MINOR(*this,0,2,3,1,2,3),  MINOR(*this,0,1,3,1,2,3), -MINOR(*this,0,1,2,1,2,3),
                           -MINOR(*this,1,2,3,0,2,3),  MINOR(*this,0,2,3,0,2,3), -MINOR(*this,0,1,3,0,2,3),  MINOR(*this,0,1,2,0,2,3),
                            MINOR(*this,1,2,3,0,1,3), -MINOR(*this,0,2,3,0,1,3),  MINOR(*this,0,1,3,0,1,3), -MINOR(*this,0,1,2,0,1,3),
                           -MINOR(*this,1,2,3,0,1,2),  MINOR(*this,0,2,3,0,1,2), -MINOR(*this,0,1,3,0,1,2),  MINOR(*this,0,1,2,0,1,2));
        }

        inline float det() const {
            return m[0] * MINOR(*this, 1, 2, 3, 1, 2, 3) -
                   m[1] * MINOR(*this, 1, 2, 3, 0, 2, 3) +
                   m[2] * MINOR(*this, 1, 2, 3, 0, 1, 3) -
                   m[3] * MINOR(*this, 1, 2, 3, 0, 1, 2);
        }

        /// Full inversion of the Transformation matrix. No assumption is made about
        /// the 4x4 matrix to optimize inversion. if the Transformation is
        /// not affine you must use this procedure to invert the matrix. For
        /// instance perspective projection can't use the fast_invert() procedure
        /// @see fast_invert()
        inline CpuTransfo full_invert() const {
            return adjoint() * (1.0f / det());
        }

        /// @return the Transformation with normalized x, y, z column vectors
        inline CpuTransfo normalized() const {
            return CpuTransfo(get_mat3().normalized(), get_translation());
        }

        inline Mat3 get_mat3() const {
            return Mat3(m[0], m[1], m[2],
                           m[4], m[5], m[6],
                           m[8], m[9], m[10]);
        }

        /// get translation part of the matrix
        /// @note same as get_org()
        inline Vec3 get_translation() const {
            return Vec3(m[3], m[7], m[11]);
        }

        /// get origine of the frame represented by the matrix
        /// @note same as get_translation()
        inline Vec3 get_org() const {
            return Vec3(m[3], m[7], m[11]);
        }

        /// Check if the vectors representing the frame are orthogonals.
        /// @warning Don't mix up this with orthogonal matrices.
        inline bool is_frame_ortho(float eps = 0.0001f) const {
            return fabsf( x().dot( y() ) ) < eps &&
                   fabsf( x().dot( z() ) ) < eps &&
                   fabsf( y().dot( z() ) ) < eps;
        }

        inline
        void print() const
        {
            printf("%f %f %f %f\n", m[0 ], m[1 ], m[2 ], m[3 ] );
            printf("%f %f %f %f\n", m[4 ], m[5 ], m[6 ], m[7 ] );
            printf("%f %f %f %f\n", m[8 ], m[9 ], m[10], m[11] );
            printf("%f %f %f %f\n", m[12], m[13], m[14], m[15] );
        }

        // -------------------------------------------------------------------------
        /// @name Static transformation generators
        // -------------------------------------------------------------------------

        static inline CpuTransfo translate(float dx, float dy, float dz){
            return CpuTransfo(1.f, 0.f, 0.f, dx,
                           0.f, 1.f, 0.f, dy,
                           0.f, 0.f, 1.f, dz,
                           0.f, 0.f, 0.f, 1.f);
        }

        static inline CpuTransfo translate(const Vec3& v){
            return CpuTransfo::translate(v.x, v.y, v.z);
        }

        static inline CpuTransfo scale(float sx, float sy, float sz){
            return CpuTransfo( sx,  0.f, 0.f, 0.f,
                            0.f,  sy, 0.f, 0.f,
                            0.f, 0.f,  sz, 0.f,
                            0.f, 0.f, 0.f, 1.f);
        }

        static inline CpuTransfo scale(const Vec3& v){
            return CpuTransfo::scale(v.x, v.y, v.z);
        }

        /// Build a uniform scaling matrix on x,y,z.
        static inline CpuTransfo scale(float s){
            return CpuTransfo::scale(s, s, s);
        }

        static inline CpuTransfo rotate(const Vec3& center,
                                     const Vec3& axis,
                                     float angle,
                                     const Mat3& frame)
        {
            CpuTransfo r(frame * Mat3::rotate(axis, angle) * frame.inverse());
            return CpuTransfo::translate(center) * r * CpuTransfo::translate(-center);
        }

        static inline CpuTransfo rotate(const Vec3& center,
                                     const Vec3& axis,
                                     float angle)
        {
            CpuTransfo r(Mat3::rotate(axis, angle));
            return CpuTransfo::translate(center) * r * CpuTransfo::translate(-center);
        }

        /// build a rotation matrix around the origin.
        /// @param axis : the <b> normalized </b> axis of rotation
        /// @param angle : rotation's angle in radian
        static inline CpuTransfo rotate(const Vec3& axis, float angle)
        {
            return CpuTransfo(Mat3::rotate(axis, angle));
        }

        static inline CpuTransfo identity(){
            return CpuTransfo(1.f, 0.f, 0.f, 0.f,
                           0.f, 1.f, 0.f, 0.f,
                           0.f, 0.f, 1.f, 0.f,
                           0.f, 0.f, 0.f, 1.f);
        }

        static inline CpuTransfo empty(){
            return CpuTransfo(0.f, 0.f, 0.f, 0.f,
                           0.f, 0.f, 0.f, 0.f,
                           0.f, 0.f, 0.f, 0.f,
                           0.f, 0.f, 0.f, 0.f);
        }

    };
}
#endif

