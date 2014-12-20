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
#ifndef RAYTRACING_HPP__
#define RAYTRACING_HPP__

#include "bbox.hpp"
#include "ray_cu.hpp"
#include "raytracing_context.hpp"
#include <math_constants.h>



/*__device__ */__constant__ float bbox___[6];

// =============================================================================
namespace Raytracing {
// =============================================================================

/// The Kernel that raytrace the given implicit object (CSG).
/// In this kernel each thread is associated to an image block of size
/// (steps.x * steps.y). The kernel only compute one pixel inside this block.
/// steps.z defines the indice of the pixel to raytrace inside the block.
/// @param potential_2d Raytrace the 2d plane potential
/// @tparam CSG the implicit tree used to draw the scene
template <class CSG>
__global__
void raytrace_kernel( const Raytracing::Context *ctx );

/// Compute the ray starting from the camera for teh current thread
__device__
Ray_cu primary_ray(const Camera_data& cam,
                   const int width,
                   const int height,
                   const int px,
                   const int py);

/// Intersection between a ray and a plane
__device__
bool inter_plane_ray(const Ray_cu& r,
                     const Point_cu& p_org,
                     const Vec3_cu& p_normal,
                     float& t);


/// Ray march along the given ray
template <class CSG, bool reverse>
__device__
float ray_march(const Ray_cu& r,
                float& step,
                int max_step,
                float tmin,
                float tmax);


/// Look for the intersection with a dichotomic search
template <class CSG, bool reverse>
__device__
float dichotomic_search(const Ray_cu& ray,
                        float t0_,
                        float t1_,
                        int max_iterations);

/// Do the ray marching + dichotomic search
template <class CSG>
__device__
bool intersect_implicit(const Ray_cu&  r,
                        float tmin,
                        float& tmax,
                        float step,
                        int nb_steps);


// -----------------------------------------------------------------------------

template <class CSG>
struct Evaluator{
#define Cs (0.5f)

    __device__
    static float f(const Point_cu& p){
        return CSG::f(p) - Cs;
    }

    __device__
    static Vec3_cu gf(const Point_cu& p){
        return CSG::gf(p);
    }

    __device__
    static float fngf(Vec3_cu& gf, const Point_cu& p){
        return CSG::fngf(p, gf) - Cs;
    }

    __device__
    static float f0(const Point_cu& p){
        return CSG::left(p) - Cs;
    }

    __device__
    static float f1(const Point_cu& p){
        return CSG::right(p) - Cs;
    }

};

// -----------------------------------------------------------------------------

/// Transform z coordinate to be compatible with openGL's depth representation
__device__
float z_transform(float z, float z_near, float z_far, bool ortho)
{
    if(z < z_near) z = z_near;
    if(z > z_far ) z = z_far;

    float dif = z_far - z_near;
    if(ortho) return (2.f  * z - (z_far + z_near)) / dif;
    else      return (z_far + z_near  - 2.f * (z_near * z_far)/z)/dif;
}

// -----------------------------------------------------------------------------

__device__
Vec3_cu light_stage(const Material_cu& mat,
                    const Camera_data& cam,
                    const Point_cu& p,
                    const Vec3_cu& grad,
                    const Ray_cu&  ray)
{
    // Point_cu light_pos0(0.f,  50.f, 0.f);
    // Point_cu light_pos1(0.f, -50.f, 0.f);
    Point_cu light_pos0(  10.f,  75.f,  20.f);
    Point_cu light_pos1( -10.f, -75.f, -20.f);
    light_pos1 = cam._pos;

    Vec3_cu light0 = (light_pos0 - p).normalized();
    Vec3_cu light1 = (light_pos1 - p).normalized();

    Vec3_cu half0 = (light0 - ray._dir).normalized();
    Vec3_cu half1 = (light1 - ray._dir).normalized();

    float lambert0 = 0.f;//fmaxf(0.f,grad.dot(light0)) * 0.5f;
    float lambert1 = fmaxf(0.f,grad.dot(light1)) * 0.5f;

    float phong0 = 0.f;//pow(fmaxf(0.f,half0.dot(grad)), mat.sh) * 0.5f;
    float phong1 = pow(fmaxf(0.f,half1.dot(grad)), mat.sh) * 0.5f;

#if 0
    float f   = Evaluator<CSG>::f(p);
    float df0 = Evaluator<CSG>::f0(p) -f;
    float df1 = Evaluator<CSG>::f1(p) -f;
    float u0  = df0 / (df0 + df1);
    float u1  = 1.f - u0;
    Vec3_cu col0(1.f, 0.1f*0+0.9, 0.1f*0+0.7);
    Vec3_cu col1(0.3f*0+0.8, 0.3f*0.8, 1.f);
    Vec3_cu col2(0.f, 1.1f, 0.f);
    Vec3_cu col = col0 * u0 * u0 + col1 * u1 * u1 + col2 * u1 * u0 * 2.f;
#else
    Vec3_cu col(1.f, 1.f, 1.f);
#endif

    return mat.Kd * (lambert0 + lambert1) + mat.Ks * (phong0 + phong1) + mat.A;
}

// -----------------------------------------------------------------------------

template <class CSG>
__device__
Vec3_cu color_2D_slice(const Raytracing::Context* ctx,
                       const Point_cu& inter,
                       const Vec3_cu& p_color,
                       float& alpha,
                       bool behind)
{
    Vec3_cu grd;
    float f = Evaluator<CSG>::fngf(grd, inter) + Cs;
    grd = -grd.normalized();

    //float d = Field::field_to_distance(f, 1.f);
    float opacity = cosf(f * 3.141592f * 15.f);
    opacity *= opacity;
    opacity  = 1.f - opacity;
    opacity *= opacity;
    opacity *= opacity;
    opacity  = 1.f - opacity;

    if( behind ) opacity = opacity * 0.35f + 0.65f;

    float opacity_inv = 1.f - opacity;
    alpha = opacity_inv;

    Vec3_cu color(0.0f, 0.9f, 0.0f);
    if(f > Cs){
        //opacity = opacity + 0.5f;
//                color = make_float3(0.9f, 0.2f, 0.1f); // red
        color = ctx->potential_colors.intern_color.to_vec3();
    }
    else if( f <= Cs  ){
//                color = make_float3(0.1f, 0.2f, 0.9f); // blue
        color = ctx->potential_colors.extern_color.to_vec3();
    }

#if 1
    if( f >= /*sqrt(2.f)*/ 1.f){
//                color = make_float3(1.0f, 0.75f, 0.5f); // brown
        color = ctx->potential_colors.huge_color.to_vec3();
    }
    else if( f < 0.0f  ){
//                color = make_float3(0.9f, 0.9f, 0.2f); // yellow
        color = ctx->potential_colors.negative_color.to_vec3();
    }
    else if( isnan(f) || grd.norm() < 0.01f  ){
        color = Vec3_cu::zero();
        opacity_inv = opacity = 0.5f;
    }
#endif

//            if (f>0.0f && f<1e-4) p_color = Vec3_cu::zero();

    float epsilon = 1e-4;
    if( f < (1.f+epsilon) && f > (1.f-epsilon))
        return Vec3_cu::zero();
    if( f < (epsilon*10) && f > 0)
        return Vec3_cu::zero();

//            p_color *= (grd * 0.5 + 0.5f);
    return p_color * opacity + color * opacity_inv;
}

// -----------------------------------------------------------------------------

template <class CSG>
__global__
void raytrace_kernel( const Raytracing::Context* ctx )
{
    const int& width  = ctx->pbo.width;
    const int& height = ctx->pbo.height;
    const Material_cu& mat = ctx->mat;
    const Camera_data& cam = ctx->cam;
    const int3& steps = ctx->steps;
    const Vec3_cu& back_cl = ctx->background.to_vec3();

#if 1
    int px = blockIdx.x*blockDim.x + threadIdx.x;
    int py = blockIdx.y*blockDim.y + threadIdx.y;

    const int size_block = (steps.x * steps.y);
    const int idx  = ((size_block/2) + steps.z) % size_block;
    const int offx = (idx % steps.x);
    const int offy = (idx / steps.x);
    px = px * steps.x + offx;
    py = py * steps.y + offy;

    if (px >= width || py >= height) return;

    const int p = py * width + px;
    const Ray_cu ray_base = primary_ray(cam, width, height, px, py);
    Ray_cu ray = ray_base;
    const float time_to_depth_factor = cam._dir.dot( ray._dir );
    const int nb_refl = ctx->nb_reflexion;
    int lvl_reflexion = nb_refl + 1;
    int step_len = ctx->step_len;


    Vec3_cu p_color = Vec3_cu::unit_scale();
    Vec3_cu color   = ctx->draw_tree ? Vec3_cu::zero() : back_cl;

    int   nb_iter = 150;
    float alpha   = 1.f;

    float first_tHit = CUDART_INF_F;
    float tHit       = CUDART_INF_F;
    float t_hit_tmp  = CUDART_INF_F;
    float tmin;
    while( (lvl_reflexion-- > 0) && ctx->draw_tree)
    {
        tmin = 0.f;
        const bool isect_test = intersect_implicit<CSG>(ray, tmin, tHit, ctx->step_len, nb_iter);
        t_hit_tmp = tHit;
        if( !isect_test ) {
            color = back_cl;
            break;
        }

        if(lvl_reflexion == nb_refl ) first_tHit = tHit;
        Point_cu p = ray(tHit-1e-5f);
        Vec3_cu grad = -Evaluator<CSG>::gf(p).normalized();
        Vec3_cu ref = ray._dir - grad * (ray._dir.dot(grad) * 2.f);

        Vec3_cu cl;
        if(ctx->enable_lighting) cl = light_stage(mat, cam, p, grad, ray);
        else                     cl = grad * 0.5f + 0.5f;

        color   += p_color * cl * alpha;
        p_color *= cl * (1.f - alpha);

        ray._pos = p + grad * 1e-4f;
        ray._dir = ref;

        //decrease precision for further reflexion rays
        step_len *= 2;
        nb_iter >>= 1;
    }

    p_color = color + p_color * (1.f - alpha);

    float t_plane = CUDART_INF_F;
    // Drawing potential plane
    if(ctx->potential_2d)
    {
        bool res = inter_plane_ray(ray_base, ctx->plane_org, ctx->plane_n, t_plane);
        if(res && t_plane > 0.f)
        {
            Point_cu inter = ray_base(t_plane);
            p_color = color_2D_slice<CSG>(ctx, inter, p_color, alpha, t_plane >= t_hit_tmp);
        }
    }

    ctx->pbo.d_rendu[p] = make_float4(p_color.x, p_color.y, p_color.z, alpha);

    // to compute depth from traversal time
    float z = time_to_depth_factor * (first_tHit < t_plane || t_plane < 0 ? first_tHit : t_plane);
    ctx->pbo.d_depth[p] = z_transform(z, cam._near, cam._far, cam._ortho) * 0.5f + 0.5f;
#endif
}

// -----------------------------------------------------------------------------

__device__
Ray_cu primary_ray(const Camera_data& cam,
                   const int width,
                   const int height,
                   const int px,
                   const int py)
{
    Ray_cu ray;
    if(cam._ortho){
        ray.set_dir(cam._dir);
        float dx = (px * 1.f / width - 0.5f) * cam._fov;
        float dy = (py * 1.f / height - 0.5f) * cam._fov * (height * MULTISAMPX *  1.f/ (width * MULTISAMPY));
        ray.set_pos(cam._pos + cam._hor * dx + cam._ver * dy);
    } else {
        Vec3_cu dep =  cam._dir * (0.5f/tanf(0.5f * cam._fov));
        ray.set_pos(cam._pos);
        Vec3_cu ver = cam._ver * (1.f / height);
        Vec3_cu hor = cam._hor * (MULTISAMPY * (1.f / (height * MULTISAMPX)));
        Vec3_cu dir = (dep + hor * (px - width/2)	+ ver * (py - height/2));
        ray.set_dir(dir.normalized());
    }
    return ray;
}

// -----------------------------------------------------------------------------

__device__
bool inter_plane_ray(const Ray_cu& r,
                     const Point_cu& p_org,
                     const Vec3_cu& p_normal,
                     float& t)
{
    const float eps = 0.00001f;
    const float denominator = p_normal.dot( r._dir );

    if(fabs(denominator) < eps)
        return false;

    float d = -(p_normal.x*p_org.x + p_normal.y*p_org.y+p_normal.z*p_org.z);
    t  = -((p_normal.x*r._pos.x + p_normal.y*r._pos.y + p_normal.z*r._pos.z) + d);
    t /= denominator;

    return true;
}

// -----------------------------------------------------------------------------

template <class CSG, bool reverse>
__device__ float
ray_march(const Ray_cu& r, float& step, int max_step, float tmin, float tmax)
{
    int mstep = (tmax-tmin)/step + 1;
    max_step = (max_step < mstep)?max_step:mstep;
    float t;
    for(int i = 0; i < max_step; i++){    // for better csg objects : max_step * 10
        t = tmin + i * step;              // for better csg objects : step / 10
        Point_cu p = r(t);
        float f = Evaluator<CSG>::f(p);
        if(reverse){
            if(f >= 0.f) return t;
        } else {
            if(f <= 0.f) return t;
        }
    }
    for(int j = 0; j < 4; j++){
        tmin = t;
        step *= 10;
        for(int i = 1; i <10; i++){
            t = tmin + i * step;
            Point_cu p = r(t);
            float f = Evaluator<CSG>::f(p);
            if(reverse){
                if(f >= 0.f) return t;
            } else {
                if(f <= 0.f) return t;
            }
        }
    }
    return -1.f;
}

// -----------------------------------------------------------------------------

#define MAX_ITER 10

template <class CSG, bool reverse>
__device__
float dichotomic_search(const Ray_cu& ray,
                        float t0_,
                        float t1_)
{
    float t0 = t0_;
    float t1 = t1_;
    float t, f;
    for(unsigned short i = 0; i < MAX_ITER; ++i){
        t = (t0 + t1) * 0.5f;
        f = Evaluator<CSG>::f( ray(t) );
        //if(f < 0.001f ) return t0;
        if(reverse){
            if(f >= 0.f) t1 = t; else t0 = t;
        } else {
            if(f <  0.f) t1 = t; else t0 = t;
        }
    }
    return t0;
}

// -----------------------------------------------------------------------------

template <class CSG>
__device__
bool intersect_implicit(const Ray_cu&  r,
                        float tmin,
                        float& tmax,
                        float step,
                        int nb_steps)
{
    float t;
    float step_   = step;

//    if( ((BBox_cu*)bbox___)[0].isect_device(r, tmin, tmax) )
//    {

        bool init_pos = (Evaluator<CSG>::f(r(tmin)) < 0.f);


        if(init_pos) t = ray_march<CSG, true> (r, step_, nb_steps, tmin, tmax);
        else         t = ray_march<CSG, false>(r, step_, nb_steps, tmin, tmax);

        if(t > 0.f){
            if(init_pos) t = dichotomic_search<CSG, true>(r,t-step_,t);
            else         t = dichotomic_search<CSG, false>(r,t-step_,t);

            if(t < tmax)
            {
                tmax = t;
                return true;
            }
        }
//    }

    return false;
}

}// END RAYTRACING =============================================================


#endif // RAYTRACING_HPP__
