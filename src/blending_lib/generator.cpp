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
#include "generator.hpp"

#include "controller_tools.hpp"
#include "controller.hpp"
#include "funcs.hpp"

#include <iostream>
#include <cassert>
#include <cmath>


#include <vector> // DEBUG
#include "splines.hpp" // DEBUG

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif


// =============================================================================
namespace IBL {
// =============================================================================

// =============================================================================
namespace Profile {
// =============================================================================


double Hyperbola::f(double x) const {
    return u(x);
}

double Hyperbola::df(double x) const {
    return du(x);
}

// HYPERBOlA END ---------------------------------------------------------------

// BULGE -----------------------------------------------------------------------

static inline int factorial(int x){
    int r = 1;
    for( int i = 1; i <= x; i++ )
        r = r * i;
    return r;
}

/// A simple integer power function
/// @return x^p
/// @note p must be positive
template<class T>
static inline
T ipow(T x, int p)
{
    assert(p >= 0);
    if (p == 0) return 1;
    if (p == 1) return x;
    return x * ipow(x, p-1);
}

////////////////////////////////////////
//BEZIER

double K = 0.1;
// TESt 0
//double p[] = {0, 0, 0, K/10, 2*K/10, 3*K/10, 4*K/10, 5*K/10, 6*K/10, 7*K/10, 8*K/10, 9*K/10, K, 0};
// Test 1
double p[] = {1, 1, 0.5, 1, 0.2, 1};
// End Test
const int size = sizeof(p);
const int n = sizeof(p)-1;

///////////////////////////////////////////
// SPLINES

const float F = 0.1f;

const float pas = 3.f/2.f/6.f;

//float init[] = {0,0,F/2, F, 3*F/2, 2*F,5*F/2,5*F/2,3*F/2,F/2,-F/2};
//const float init[] = {0.f, 0.f, 0.f, pas*F, 2.f*pas*F, 3.f*pas*F, 4.f*pas*F, 5.f*pas*F, 3.f*F/2.f, F/2.f, -F/2.f};
const float init[] = {0.f, 0.f, 0.f, pas*F, 2.f*pas*F, 3.f*pas*F, 4.f*pas*F, 5.f*pas*F, 4.f*pas*F, 4.f*pas*F, 4.f*pas*F};
std::vector<float> points(init, init + (sizeof(init) / sizeof(int)) );

Spline<float, float> spline(3, ESpline::UNIFORM);

// -----------------------------------------------------------------------------

double Bulge::f(double t) const
{
#if 0
    // original gradient based
    return 1. - exp(1. - (1./t)) * (1. - t)*_magnitude;
#elif 0
    // skinning bezier
    double y = 0;
    for(int i = 0; i < size; i++){
        const double coef_bin = factorial(n)/(factorial(i) * factorial(n-i));
        y = y + coef_bin * ipow(t, i) * ipow( 1-t, (n-i) ) * p[i];
    }
    return y;
#else
    // skinning splines
    std::vector<float> tmp = points;
    for(unsigned i = 0; i < points.size(); ++i) tmp[i] = 1.f - tmp[i];
    spline.set_ctrl_points(tmp);
    float val = spline.eval_f((float)t);
    return val;
#endif
}

// -----------------------------------------------------------------------------

double Bulge::df(double t) const
{
#if 0
    // original gradient based
    if(t <= 0.)
        return 0.;
    else
        return (-_magnitude * exp(1. - (1./t)) * ((1. - t)/(t*t) - 1.));
#elif 0
    // skinning bezier
    if(t <= 0.) return 0.;

    double y = 0;
    for(int i = 0; i < (size-1); i++){
        const int dn = n-1;
        const double coef_bin = factorial(dn)/(factorial(i) * factorial(dn-i));
        y = y + coef_bin * ipow(t, i) * ipow( 1-t, (dn-i) ) * (p[i+1] - p[i]);
    }
    y *= n;
    return y;
#else
    // skinning splines
    std::vector<float> tmp = points;
    for(unsigned i = 0; i < points.size(); ++i) tmp[i] = 1.f - tmp[i];
    spline.set_ctrl_points(tmp);
    return spline.eval_df((float)t);

#endif
}

// BULGE END -------------------------------------------------------------------

// RICCI -----------------------------------------------------------------------
Ricci_profile::Ricci_profile(double n) :
    _n( n < 0.1 ? 0.1 : n ) // Avoid dividing by zero
{  }

double Ricci_profile::f(double x) const {
    return pow( 1. - pow(x, _n) , 1./_n);
}

double Ricci_profile::df(double x) const{
    return - pow(x, _n-1.) * pow( 1. - pow(x, _n) , 1./_n - 1.);
}
// RICCI END -------------------------------------------------------------------

// Spline ----------------------------------------------------------------------

Spline_profile::Spline_profile(const Spline<Vec2_cu, float>& sp) :
    _spline(sp)
{ }

// -----------------------------------------------------------------------------

double Spline_profile::f(double x) const
{
    float a = 0.f, b = 1.f, u = 0.5f;
    for(int i = 0; i < 15; ++i)
    {
        if(_spline.eval_f(u).x * 2.f < x)
            a = u;
        else
            b = u;
        u = (a+b)/2;
    }

    return _spline.eval_f(u).y * 2.f;
}

// -----------------------------------------------------------------------------

double Spline_profile::df(double x) const
{
    float a = 0.f, b = 1.f, u = 0.5f;
    for(int i = 0; i < 15; ++i)
    {
        if( (_spline.eval_f(u).x * 2.f) < x)
            a = u;
        else
            b = u;
        u = (a+b) / 2.f;
    }

    return _spline.eval_df(u).y * 2.f;
}

// Spline END ------------------------------------------------------------------

}
// =============================================================================


// =============================================================================
namespace Profile_polar {
// =============================================================================

// -----------------------------------------------------------------------------

float Discreet::f(float tan_t) const {
    assert(_nb_samples > 0);
    return linear_fetch(tan_t);
}

IBL::float2 Discreet::gf(float ) const {
    // TODO:
    assert(false);
    assert(_nb_samples > 0);
    return IBL::make_float2(0.f, 0.f);
}

float Discreet::linear_fetch(float tan_t) const
{
    if(tan_t < 0.f)
        return sqrtf(1.f + tan_t*tan_t);

    tan_t = std::min(1.f,tan_t);
    float u = (_nb_samples - 1)*tan_t;
    int i = (int)floor(u);

    assert( !(i < 0 || i > (_nb_samples - 1)) );

    float v0 = _vals[i];
    if(i >= (_nb_samples - 1))
        return v0;
    float v1 = _vals[i+1];
    float dt = u-i;
    return dt * v1 + (1.f - dt) * v0;
}

// -----------------------------------------------------------------------------

float Circle::f(float ) const {
    return 1.f;
}

IBL::float2 Circle::gf(float tan_t) const{
    IBL::float2 grad = IBL::make_float2(1.f, tan_t);
    return IBL::normalized(grad);
}

// -----------------------------------------------------------------------------

}// END Profile ================================================================

// -----------------------------------------------------------------------------

void gen_polar_profile(Profile_polar::Discreet& discreet_curve,
                       const int nb_samples,
                       const Profile::Base& curve )
{
    const int size = nb_samples * 10;

    // Array subscripts index correspond to the function's 'func' parameter 't'
    float*       dist      = new float      [size]; // Dist to curve
    IBL::float2* normal    = new IBL::float2[size]; // Normal to curve
    float*       tan_theta = new float      [size]; // tan_theta for each sample of dist and normal

    // Fill arrays dist, normal and tan_theta
    for(int i = 0; i < size; i++)
    {
        float x  = i / (float)size;
        float f  = (float)curve.f(x);
        float df = (float)curve.df(x);

        float tan_t = x / f;
        dist[i] = sqrtf(f*f + x*x);

        float dx   = -df;
        float dy   = 1.f;
        float norm = sqrtf(dx*dx + dy*dy);

        normal   [i] = IBL::make_float2(dx / norm, dy / norm);
        tan_theta[i] = tan_t;
    }

    float*       pdist   = new float      [nb_samples]; // Distance from curve in polar
    IBL::float2* pnormal = new IBL::float2[nb_samples]; // Normal to curve in polar

    // Fill pdist and pnormal
    bool gen_ok = true;
    int j = 0;
    for(int i = 0; i < nb_samples; i++)
    {
        float tan_t = i / (float)(nb_samples-1);
        // Searching for the index of the angles j and j+1 in between tan_t
        while( (tan_theta[j+1] < tan_t) && (j < size-2) )
        {
            j++;
        }

        float tan0 = tan_theta[j  ]; // > tan_t
        float tan1 = tan_theta[j+1]; // < tan_t
        // Linearly interpolate distance and normals between tan0 and tan1
        float lmb = (tan1 - tan_t) / (tan1 - tan0);
        float mean_dist = lmb * dist  [j]   + (1.f - lmb) * dist  [j+1];
        float mean_nx   = lmb * normal[j].x + (1.f - lmb) * normal[j+1].x;
        float mean_ny   = lmb * normal[j].y + (1.f - lmb) * normal[j+1].y;

        IBL::float2 mean_normal = IBL::make_float2(mean_nx, mean_ny);
        mean_normal = IBL::normalized(mean_normal);

        IBL::float2 arc_dir = IBL::make_float2(tan_t, 1.f);
        arc_dir = IBL::normalized(arc_dir);
        float dot_prod = IBL::dot(arc_dir, mean_normal);

        if(dot_prod < 0.f) gen_ok = false;

        float norm = 1.f / (dot_prod * mean_dist);
        pdist  [i] = mean_dist;
        pnormal[i] = IBL::mult(mean_normal, norm);
    }

    delete[] dist;
    delete[] normal;
    delete[] tan_theta;

    if( !gen_ok ){
        std::cerr << "ERROR: can't generate this kind of profile" << std::endl;
    }

    discreet_curve = Profile_polar::Discreet(pdist, pnormal, nb_samples);
}

// -----------------------------------------------------------------------------

void gen_custom_operator(const Profile_polar::Base& profile,
                         const Opening::Base& opening,
                         double range,
                         int nb_samples_ocu,
                         int nb_samples_alpha,
                         float*& out_values,
                         IBL::float2*& out_gradients)
{
    const int size = nb_samples_ocu*nb_samples_ocu*nb_samples_alpha;
    double*       values   = new double      [size];
    IBL::double2* gradient = new IBL::double2[size];

    // Init arrays
    for(int i = 0; i < size; i++){
        values  [i] = -1.;
        gradient[i] = IBL::make_double2(0., 0.);
    }

    //float step_i = range / (float)(nb_samples_ocu-1);
    for(int alpha = 0; alpha < nb_samples_alpha; alpha++) // Fill values by opening angles
    {
        int offset = alpha * nb_samples_ocu*nb_samples_ocu;
        values[offset] = 0.;
        double tan_alpha = alpha / (double)(nb_samples_alpha-1.);

        for(int i = 0; i < nb_samples_ocu; i++)
        {
            double x = ((double)i * range) / (double)(nb_samples_ocu-1);
            double x2 = opening.f(x, tan_alpha);

            for(int j = 0; j < ((x2 * (double)(nb_samples_ocu-1)) / range); j++){
                assert((i + j*nb_samples_ocu + offset) < size);
                assert((j + i*nb_samples_ocu + offset) < size);

                values[i + j*nb_samples_ocu + offset] = x;
                values[j + i*nb_samples_ocu + offset] = x;
            }
            //printf("i = %d\n",i);fflush(stdout);
            double c0 = x2;
            double xtmp = ((double)(i+1) * range) / (double)(nb_samples_ocu-1);
            double c1 = opening.f(xtmp, tan_alpha);

            int k0 = (int)floor( ((x2 * (double)(nb_samples_ocu-1)) / range) ) /* x2 * (nb_samples_ocu-1)*/;
            int k1 = i;
            for(int ik = k0; ik <= k1; ik++){
                double xk = ((double)ik * range) / (double)(nb_samples_ocu-1);
                for(int jk = k0; jk <= k1; jk++){
                    double yk = ((double)jk * range) / (double)(nb_samples_ocu-1);
                    double dx = xk - c0;
                    double dy = yk - c0;
                    double tan0 = (dx<dy)?dx/dy:(dy/dx);

                    assert((ik + jk*nb_samples_ocu + offset) < size);
                    if(values[ik + jk*nb_samples_ocu + offset] == -1.)
                    {
                        if(tan0 < 0.){
                            values[ik + jk*nb_samples_ocu + offset] = (dx<dy)?yk:xk;
                        } else {
                            double r0 = sqrt(dx*dx + dy*dy);
                            r0 /= profile.f(tan0);
                            dx = xk - c1;
                            dy = yk - c1;
                            double tan1 = (dx<dy)?dx/dy:(dy/dx);
                            double r1 = sqrt(dx*dx + dy*dy);
                            r1 /= profile.f(tan1);

                            if( (r0 >= (x - c0)) & (r1 <  (xtmp - c1)))
                            {
                                double d0 = r0 - (x - c0);
                                double d1 = (xtmp - c1) - r1;
                                double lbd = d1 / (d1 + d0);

                                values[ik + jk*nb_samples_ocu + offset] = lbd * x + (1. - lbd) * xtmp;
                            }
                        }
                    }

                }
            }
        }

        // Building isos which are not connected to a max
        double org = opening.f(range, tan_alpha);
        int   p0  = (int)floor( ((org * (double)(nb_samples_ocu-1)) / range) );
        for(int i = p0; i < nb_samples_ocu; i++){
            double xi = ((double)i * range) / (double)(nb_samples_ocu-1);
            double dx = xi - org;
            for(int j = p0; j < nb_samples_ocu; j++){
                assert((i + j*nb_samples_ocu + offset) < size);
                if(values[i + j*nb_samples_ocu + offset]==-1.){
                    double xj = ((double)j * range) / (double)(nb_samples_ocu-1);
                    double dy = xj - org;
                    double r = sqrt(dx*dx + dy*dy);
                    double tant = (dx<dy) ? (dx/dy) : (dy/dx);
                    r /= profile.f(tant);
                    values[i + j*nb_samples_ocu + offset] = r + org;
                }
            }
        }

        // Smoothing values
#if 1
        for(int i = 0; i < nb_samples_ocu; i++){
            for(int j = 0; j < nb_samples_ocu; j++){
                assert((i + j * nb_samples_ocu + offset) < size);
                double v = values[i + j *nb_samples_ocu + offset];
                if(v == -1.)
                {
                    //printf("%d %d %d\n",alpha,i,j);
                    double acc = 0.;
                    int nb = 0;
                    assert(((i-1) + j*nb_samples_ocu + offset) < size);
                    double v0 = values[i-1 + j *nb_samples_ocu + offset];
                    if(v0 > -1.){
                        acc += v0;
                        nb++;
                    }
                    assert(((i+1) + j*nb_samples_ocu + offset) < size);
                    v0 = values[i+1 + j *nb_samples_ocu + offset];
                    if(v0 > -1.){
                        acc += v0;
                        nb++;
                    }
                    assert((i + (j-1)*nb_samples_ocu + offset) < size);
                    v0 = values[i + (j-1)*nb_samples_ocu + offset];
                    if(v0 > -1.){
                        acc += v0;
                        nb++;
                    }
                    assert((i + (j+1)*nb_samples_ocu + offset) < size);
                    v0 = values[i + (j+1)*nb_samples_ocu + offset];
                    if(v0 > -1.){
                        acc += v0;
                        nb++;
                    }
                    values[i + j *nb_samples_ocu + offset] = acc/nb;
                }
            }
        }
#endif


        //compute gradient with finite differences
        for(int i = 1; i < nb_samples_ocu-1; i++)
        {
            for(int j = 1; j< nb_samples_ocu-1; j++)
            {
                double dfx = values[i+1 + j*nb_samples_ocu + offset] -
                            values[i-1 + j*nb_samples_ocu + offset];

                double dfy = values[i + (j+1)*nb_samples_ocu + offset]-
                            values[i + (j-1)*nb_samples_ocu + offset];

                double dl = (2. * range) / (double)(nb_samples_ocu-1);

                IBL::double2 gf = IBL::make_double2(dfx / dl, dfy / dl);
                gradient[i + j * nb_samples_ocu + offset] = gf;
            }
        }

        for(int i = 1; i < nb_samples_ocu-1; i++)
        {

            double dy = values[nb_samples_ocu*nb_samples_ocu-1  -i   + offset] -
                       values[nb_samples_ocu*(nb_samples_ocu-1)-1-i + offset];

            double dx = values[nb_samples_ocu*nb_samples_ocu  -i   + offset] -
                       values[nb_samples_ocu*nb_samples_ocu-2-i + offset];

            IBL::double2 gf = IBL::make_double2(dx * 0.5 * (nb_samples_ocu - 1), dy *(nb_samples_ocu - 1));
            gradient[nb_samples_ocu*nb_samples_ocu-1-i + offset] = gf;

            dx = values[nb_samples_ocu*nb_samples_ocu-1-i*nb_samples_ocu + offset] -
                 values[nb_samples_ocu*nb_samples_ocu-2-i*nb_samples_ocu + offset];

            dy = values[nb_samples_ocu*(nb_samples_ocu+1)-1-i*nb_samples_ocu + offset] -
                 values[nb_samples_ocu*(nb_samples_ocu-1)-1-i*nb_samples_ocu + offset];

            gf = IBL::make_double2(dx * (nb_samples_ocu - 1), dy * 0.5 * (nb_samples_ocu - 1));
            gradient[nb_samples_ocu*nb_samples_ocu-1-i*nb_samples_ocu + offset] = gf;

            gradient[i                + offset] = IBL::make_double2(1.,0.);
            gradient[i*nb_samples_ocu + offset] = IBL::make_double2(0.,1.);
        }

        //gradient values at corners
        gradient[nb_samples_ocu-1                  + offset] = IBL::make_double2(1.,0.);
        gradient[(nb_samples_ocu-1)*nb_samples_ocu + offset] = IBL::make_double2(0.,1.);
        gradient[nb_samples_ocu*nb_samples_ocu-1   + offset] = IBL::make_double2(0.620133, 0.620133);

        //printf("\r %f per cents done",(alpha+1)*100.f/NB_SAMPLES_ALPHA);fflush(stdout);//-------------

    } // END FOR( NB_SAMPLES_ALPHA )

    out_values    = new float      [size];
    out_gradients = new IBL::float2[size];

    // Init arrays
    for(int i = 0; i < size; i++){
        out_values   [i]   = (float)values  [i];
        out_gradients[i].x = (float)gradient[i].x;
        out_gradients[i].y = (float)gradient[i].y;
    }
    delete [] values;
    delete [] gradient;
}

// -----------------------------------------------------------------------------

void gen_controller(int nb_samples,
                    const Ctrl_setup& shape,
                    IBL::float2*& out_values)
{
    assert(nb_samples > 0);
    IBL::float2* values = new IBL::float2[nb_samples];
    float b0 = shape.p0().x;
    float b1 = shape.p1().x;
    float b2 = shape.p2().x;
    float F0 = shape.p0().y;
    float F1 = shape.p1().y;
    float F2 = shape.p2().y;
    float S0 = shape.s0();
    float S1 = shape.s1();

    for(int i = 0; i < nb_samples; i++)
    {
        float t = i * 1.f / (nb_samples - 1);
        float dot = 2.f * t - 1.f;
        float fdot;
        float derv;
        if(dot < b1){
            if(dot < b0){
                fdot = F0;
                derv = 0.f;
            } else {
                float a = 1.f/(b0-b1);
                float b = - b1 * a;
                fdot = signeg(a * dot + b, S0);
                derv = a*dsig(a * dot + b, S0);
                if(b0 < -1.f){
                    fdot /= signeg(b - a, S0);
                    derv /= signeg(b - a, S0);
                }
                fdot = (F0 - F1) * fdot  + F1;
                derv = (F0 - F1) * derv;
            }
        } else {
            if(dot > b2){
                fdot = F2;
                derv = 0.f;
            } else {
                derv = 0.f;
                float a = 1.f/(b2-b1);
                float b = - b1 * a;
                fdot = sigpos(a * dot + b, S1);
                if(b2 > 1.f)
                    fdot /= sigpos(a + b, S1);
                fdot = (F2 - F1) * fdot + F1;

            }
        }
        float alpha = fdot * M_PI * 0.2499f + 0.00001f;
        float tan_alpha = tanf(alpha);
        float iterp_factor = fdot * fdot;
        iterp_factor *= iterp_factor;
        iterp_factor *= iterp_factor;
        values[i] = IBL::make_float2(tan_alpha, 0.03f * sqrt(1.f + derv*derv));
    }

    out_values = values;
}

// -----------------------------------------------------------------------------



}
// END IBL =====================================================================

