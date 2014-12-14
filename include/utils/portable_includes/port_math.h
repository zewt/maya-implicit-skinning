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
#ifndef PORT_MATH_H_
#define PORT_MATH_H_

/**
    This file is designed to provide a portable interface for cmath lib
    between compilers.
*/

/// Include standard math librairie
#if defined(__cplusplus)
#include <cmath>
#else
#include <math.h>
#endif

/// Because visual studio versions lower than the 2010 suit does not
/// comply with C99 standard we have to implement by hand min max functions
#if (_MSC_VER < 1600) && !defined(__CUDACC__) &&  !defined(__MINGW32__)

/// If windows.h is included and macro NOMINMAX is not defined definitions of
/// min/max functions in this header will results in name conflicts. This is so
/// because windows.h defines min max  functions unless NOMINMAX is defined
#if defined(_WINDOWS_) && !defined(NOMINMAX)
#error ---- When "windows.h" is used with "port_math.h" macro NOMINMAX must be defined ! ----
#endif

/// We need some compiler specific macros to inline functions
/// This is specific to visual studio
#define __inline__ \
        __inline

#if defined(__cplusplus)
extern "C" {
#endif

    /// Implementation comes from NVIDIA nvcc compiler header : "math_functions.h"
    /// You can find it by dowloading the cuda toolkit at :
    /// http://developer.nvidia.com/object/cuda_3_2_downloads.html

    //extern   int                    min(int, int);
    extern   int           umin(int, int);
    extern   long long int          llmin(long long int, long long int);
    extern   unsigned long long int ullmin(unsigned long long int, unsigned long long int);
    extern   float                  fminf(float, float);
    extern   double                 fmin(double, double);
    //extern   int                    max(int, int);
    extern   int           umax(int, int);
    extern   long long int          llmax(long long int, long long int);
    extern   unsigned long long int ullmax(unsigned long long int, unsigned long long int);
    extern   float                  fmaxf(float, float);
    extern   double                 fmax(double, double);

#if defined(__cplusplus)
} // extern "C"
#endif

// IMPLEMENTATION ==========================================================

static __inline__ float fmax(float a, float b)
{
    return fmaxf(a, b);
}
static __inline__ float fmin(float a, float b)
{
    return fminf(a, b);
}
static __inline__   int min(int a, int b)
{
    return umin(a, b);
}
static __inline__   int min(int a, int b)
{
    return umin((int)a, b);
}
static __inline__   int min(int a, int b)
{
    return umin(a, (int)b);
}
static __inline__   long long int min(long long int a, long long int b)
{
    return llmin(a, b);
}
static __inline__   unsigned long long int min(unsigned long long int a, unsigned long long int b)
{
    return ullmin(a, b);
}
static __inline__   unsigned long long int min(long long int a, unsigned long long int b)
{
    return ullmin((unsigned long long int)a, b);
}
static __inline__   unsigned long long int min(unsigned long long int a, long long int b)
{
    return ullmin(a, (unsigned long long int)b);
}
static __inline__   float min(float a, float b)
{
    return fminf(a, b);
}
static __inline__   double min(double a, double b)
{
    return fmin(a, b);
}
static __inline__   double min(float a, double b)
{
    return fmin((double)a, b);
}
static __inline__   double min(double a, float b)
{
    return fmin(a, (double)b);
}
static __inline__   int max(int a, int b)
{
    return umax(a, b);
}
static __inline__   int max(int a, int b)
{
    return umax((int)a, b);
}
static __inline__   int max(int a, int b)
{
    return umax(a, (int)b);
}
static __inline__   long long int max(long long int a, long long int b)
{
    return llmax(a, b);
}
static __inline__   unsigned long long int max(unsigned long long int a, unsigned long long int b)
{
    return ullmax(a, b);
}
static __inline__   unsigned long long int max(long long int a, unsigned long long int b)
{
    return ullmax((unsigned long long int)a, b);
}
static __inline__   unsigned long long int max(unsigned long long int a, long long int b)
{
    return ullmax(a, (unsigned long long int)b);
}
static __inline__   float max(float a, float b)
{
    return fmaxf(a, b);
}
static __inline__   double max(double a, double b)
{
    return fmax(a, b);
}
static __inline__   double max(float a, double b)
{
    return fmax((double)a, b);
}
static __inline__   double max(double a, float b)
{
    return fmax(a, (double)b);
}

static int __isnan(double a)
{
    volatile union {
        double                 d;
        unsigned long long int l;
    } cvt;

    cvt.d = a;

    return cvt.l << 1 > 0xffe0000000000000ull;
}

static int __signbit(double a)
{
    volatile union {
        double               d;
        signed long long int l;
    } cvt;

    cvt.d = a;
    return cvt.l < 0ll;
}

static double fmax(double a, double b)
{
    if (__isnan(a) && __isnan(b)) return a + b;
    if (__isnan(a)) return b;
    if (__isnan(b)) return a;
    if ((a == 0.0) && (b == 0.0) && __signbit(b)) return a;
    return a > b ? a : b;
}

static double fmin(double a, double b)
{
    if (__isnan(a) && __isnan(b)) return a + b;
    if (__isnan(a)) return b;
    if (__isnan(b)) return a;
    if ((a == 0.0) && (b == 0.0) && __signbit(a)) return a;
    return a < b ? a : b;
}

static float fmaxf(float a, float b)
{
    return (float)fmax((double)a, (double)b);
}

static float fminf(float a, float b)
{
    return (float)fmin((double)a, (double)b);
}

static int min(int a, int b)
{
    return a < b ? a : b;
}

static int umin(int a, int b)
{
    return a < b ? a : b;
}

static long long int llmin(long long int a, long long int b)
{
    return a < b ? a : b;
}

static unsigned long long int ullmin(unsigned long long int a, unsigned long long int b)
{
    return a < b ? a : b;
}

static int max(int a, int b)
{
    return a > b ? a : b;
}

static int umax(int a, int b)
{
    return a > b ? a : b;
}

static long long int llmax(long long int a, long long int b)
{
    return a > b ? a : b;
}

static unsigned long long int ullmax(unsigned long long int a, unsigned long long int b)
{
    return a > b ? a : b;
}

// END IMPLEMENTATION ======================================================

#endif //(_MSC_VER < 1600) && !defined(__CUDACC__) &&  !defined(__MINGW32__)

#endif // PORT_MATH_H_
