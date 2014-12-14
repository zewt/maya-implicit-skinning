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
#include "trackball.hpp"

#include <cmath>
#include <cassert>


// -----------------------------------------------------------------------------

static void vset(float* v, float x, float y, float z)
{
    v[0] = x;
    v[1] = y;
    v[2] = z;
}

// -----------------------------------------------------------------------------

static void vsub(const float* src1, const float* src2, float* dst)
{
    dst[0] = src1[0] - src2[0];
    dst[1] = src1[1] - src2[1];
    dst[2] = src1[2] - src2[2];
}

// -----------------------------------------------------------------------------

static void vcopy(const float* v1, float* v2)
{
    register int i;
    for (i = 0 ; i < 3 ; i++)
        v2[i] = v1[i];
}

// -----------------------------------------------------------------------------

static void vcross(const float* v1, const float* v2, float* cross)
{
    float temp[3];

    temp[0] = (v1[1] * v2[2]) - (v1[2] * v2[1]);
    temp[1] = (v1[2] * v2[0]) - (v1[0] * v2[2]);
    temp[2] = (v1[0] * v2[1]) - (v1[1] * v2[0]);
    vcopy(temp, cross);
}

// -----------------------------------------------------------------------------

static float vlength(const float* v)
{
    return sqrtf(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

// -----------------------------------------------------------------------------

void TrackBall::roll(float a[3], float& phi, float px, float py) const
{

    const float w = (float)_width;
    const float h = (float)_height;

    const float cx = (2.f * _cx / w) - 1.f;
    const float cy = (2.f * _cy / h) - 1.f;

    const float p1x = ((2.f * _pick_x / w) - 1.f) - cx;
    const float p1y = ((2.f * _pick_y / h) - 1.f) - cy;

    const float p2x = ((2.f * px / w) - 1.f) - cx;
    const float p2y = ((2.f * py / h) - 1.f) - cy;

    TrackBall::roll(a, phi, _rad, p1x, p1y, p2x, p2y);
}

// -----------------------------------------------------------------------------

static float tb_project_to_sphere(float r, float x, float y)
{
    float n = sqrt( x * x + y * y );
    if(n < r * 0.70710678118654752440f)
        return sqrt( r * r - n * n);
    else
    {
        // On hyperbola
        float t = r / 1.41421356237309504880f;
        return t * t / n;
    }
}

// -----------------------------------------------------------------------------

void TrackBall::roll(float a[3], float& phi, float rad, float p1x, float p1y, float p2x, float p2y)
{
    float p1[3], p2[3], d[3];
    float t;

    if (p1x == p2x && p1y == p2y) {
        // Zero rotation
        phi = 0.f;
        return;
    }

    // First, figure out z-coordinates for projection of P1 and P2 to
    // deformed sphere
    vset(p1,p1x,p1y,tb_project_to_sphere(rad,p1x,p1y));
    vset(p2,p2x,p2y,tb_project_to_sphere(rad,p2x,p2y));

    // Now, we want the cross product of P1 and P2
    vcross(p1,p2,a);

    // Figure out how much to rotate around that axis.
    vsub(p1,p2,d);
    t = vlength(d) / (2.0f * rad);

    // Avoid problems with out-of-control values...
    if (t >  1.0) t =  1.0f;
    if (t < -1.0) t = -1.0f;
    phi = 2.0f * asin(t);
}

// -----------------------------------------------------------------------------
