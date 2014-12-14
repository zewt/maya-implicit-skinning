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
#ifndef BLOB_HPP
#define BLOB_HPP

#include "point_cu.hpp"
#include "vec3_cu.hpp"
#include "mat3_cu.hpp"

#include "bbox.hpp"
#include <iostream>

#include "distance_field.hpp"

class Blob {
    Point_cu mCentre;
    float mRadius;
    float mSquaredRadius;
    float mAlpha;
    BBox_cu mBBox;

    // attributes linked to operators
    float mCani_alpha;
    float mCani_w;
public:

    __device__ __host__
    Blob() :
        mCentre(Point_cu()), mRadius(1), mSquaredRadius(1), mAlpha(2), mCani_alpha(1.f), mCani_w(1.f) {
        mBBox = BBox_cu(mCentre-Vec3_cu(mRadius, mRadius, mRadius), mCentre+Vec3_cu(mRadius, mRadius, mRadius));
    }

    __device__ __host__
    Blob(Point_cu c, float r, float a) :
        mCentre(c), mRadius(r), mSquaredRadius(r*r), mAlpha(a), mCani_alpha(1.f), mCani_w(1.f) {
        mBBox = BBox_cu(mCentre-Vec3_cu(mRadius, mRadius, mRadius), mCentre+Vec3_cu(mRadius, mRadius, mRadius));
    }

    __device__ __host__
    Blob(const Blob &b) :
        mCentre(b.mCentre), mRadius(b.mRadius), mSquaredRadius(b.mSquaredRadius), mAlpha(b.mAlpha), mCani_alpha(b.mCani_alpha), mCani_w(b.mCani_w) {
        mBBox = BBox_cu(mCentre-Vec3_cu(mRadius, mRadius, mRadius), mCentre+Vec3_cu(mRadius, mRadius, mRadius));
    }

    __device__ __host__
    float f(const Point_cu &p) const {
        if ( (mCentre-p).norm_squared() <= mSquaredRadius )
            return powf( 1-(mCentre-p).norm_squared()/mSquaredRadius , mAlpha );
        else return 0;
    }

    __device__ __host__
    Vec3_cu gf(const Point_cu &p) const {
        return grad(p);
    }

    __device__ __host__
    float fngf(Vec3_cu &gf, const Point_cu &p) const {
        gf = grad(p);
        return f(p);
    }

    __device__ __host__
    Vec3_cu grad(const Point_cu &p) const {
        // x => 2*a/r² * (Xc-Xp) * (1- (Xc-Xp)²+(Yc-Yp)²+(Zc-Zp)² )/r²)^a-1
        float diff = 1-((p-mCentre).norm_squared())/mSquaredRadius;
        float a_diff__a_1 = 2 * mAlpha * powf(diff, mAlpha-1) / mSquaredRadius;
        float gradX = a_diff__a_1 * (mCentre.x - p.x);
        float gradY = a_diff__a_1 * (mCentre.y - p.y);
        float gradZ = a_diff__a_1 * (mCentre.z - p.z);
        return Vec3_cu(gradX, gradY, gradZ);
    }

    __device__ __host__
    bool intersectBlob(const Blob &other){
        return f(other.mCentre + ( mCentre-other.mCentre).normalized()*
                                   other.getSurfaceRadius() ) > 0.5f;
//        return f((mCentre + other.mCentre)*0.5f) > 0.5f;
    }

    __device__ __host__
    float getSurfaceRadius() const {
        return sqrtf( mSquaredRadius*(1-powf( 0.5 , 1/mAlpha )) );
    }

    __host__
    void rotate(const Point_cu& p, const Vec3_cu& axis_, float angle){
        Vec3_cu axis = axis_.normalized();
        Mat3_cu rot = Mat3_cu::rotate(axis, angle);
        Vec3_cu d = mCentre - p;
        mCentre = p + rot * d;
    }

    __host__
    void translate(const Vec3_cu& v){
        mCentre = mCentre + v;
        mBBox = BBox_cu(mCentre+Vec3_cu(mRadius, mRadius, mRadius), mCentre+Vec3_cu(mRadius, mRadius, mRadius));
    }

    __host__
    void set_pos(const Point_cu& p){
        mCentre = p;
        mBBox = BBox_cu(mCentre-Vec3_cu(mRadius, mRadius, mRadius), mCentre+Vec3_cu(mRadius, mRadius, mRadius));
    }

    __device__ __host__
    Point_cu get_pos() const{
        return mCentre;
    }

    __host__
    void set_radius( float r ){
        mSquaredRadius = r*r;
        mRadius = r;
        mBBox = BBox_cu(mCentre-Vec3_cu(mRadius, mRadius, mRadius), mCentre+Vec3_cu(mRadius, mRadius, mRadius));
    }

    __host__
    float get_radius(){
        return mRadius;
    }

    __host__
    void set_alpha( float a ){
        mAlpha = a;
    }

    __host__
    float get_alpha(){
        return mAlpha;
    }

    __device__ __host__
    BBox_cu getBBox() const {
        return mBBox;
    }

    // methods linked to operators
    __device__ __host__
    void setCaniAlpha( float a ){
        mCani_alpha = a;
    }

    __device__ __host__
    float getCaniAlpha (){
        return mCani_alpha;
    }

    __device__ __host__
    void setCaniW( float w ){
        mCani_w = w;
    }

    __device__ __host__
    float getCaniW (){
        return mCani_w;
    }

private:
    /* blob as a sphere => normal computed directly */
    __device__ __host__
    Vec3_cu normale(const Point_cu &p) const {
        if ( (mCentre-p).norm_squared() <= mSquaredRadius )
            return (p-mCentre).normalized();
        else
            return Vec3_cu();
    }
};

class S_Blob {
    Point_cu mCentre; ///< center
    float mRadius;    ///< surface radius
    float mOffset;    ///< potential offset around center

public:

    __device__ __host__
    S_Blob() :
        mCentre(Point_cu()), mRadius(1), mOffset(mRadius)
    { }

    __device__ __host__
    S_Blob(Point_cu c, float r, float o) :
        mCentre(c), mRadius(r), mOffset(mRadius)
    { }

    __device__ __host__
    S_Blob(const S_Blob &b) :
        mCentre(b.mCentre), mRadius(b.mRadius), mOffset(b.mOffset)
    { }

    __device__ __host__
    float f(const Point_cu &p) const {
        return Field::distance_to_field_flatten((mCentre-p).norm(), mRadius, mOffset);
    }

    __device__ __host__
    Vec3_cu gf(const Point_cu &p) const {
        float df = Field::field_derivative_from_distance_flatten((mCentre-p).norm(), mRadius, mOffset);
        Vec3_cu grad_d = (p - mCentre).normalized();
        return grad_d * df;
    }

    __device__ __host__
    float fngf(Vec3_cu &gf, const Point_cu &p) const {
        float2 fndf = Field::distance_to_field_and_derivative_flatten((mCentre-p).norm(), mRadius, mOffset);
        Vec3_cu grad_d = (p - mCentre).normalized();
        gf =  grad_d * fndf.y;
        return fndf.x;
    }

    __host__
    void rotate(const Point_cu& p, const Vec3_cu& axis_, float angle){
        Vec3_cu axis = axis_.normalized();
        Mat3_cu rot = Mat3_cu::rotate(axis, angle);
        Vec3_cu d = mCentre - p;
        mCentre = p + rot * d;
    }

    __host__
    void translate(const Vec3_cu& v){
        mCentre = mCentre + v;
    }

    __host__
    void set_pos(const Point_cu& p){
        mCentre = p;
    }

    __device__ __host__
    Point_cu get_pos() const{
        return mCentre;
    }

    __host__
    void set_radius( float r ){
        mRadius = r;
        if (mOffset > mRadius)
            mOffset = mRadius;
    }

    __host__
    float get_radius(){
        return mRadius;
    }

    __host__
    float get_potential_radius(){
        return mRadius + mOffset;
    }

    __host__
    void set_offset( float o ){
        mOffset = o;
    }

    __host__
    float get_offset(){
        return mOffset;
    }
};

#endif // BLOB_HPP







