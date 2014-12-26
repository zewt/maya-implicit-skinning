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
#include "intersection.hpp"
#include "inter_tri_tri.inl"

// =============================================================================
namespace Inter {
// =============================================================================

IF_CUDA_DEVICE_HOST static inline
bool plane_line(const Plane& p, const Line& l, Point& res)
{

    /*
        Equation du plan :
            a*x + b*y + c*z + d = 0

        normale du plan : (a b c)
        param�ｽtre d = ( -a xa -b ya -c za) avec (xa ya za) un point du plan

        Equation param�ｽtrique de la droite de param�ｽtre t
            x = x1 + i*t
            y = y1 + j*t
            z = z1 + k*t

        Vecteur directeur (i j k)
        Point de la droite (x1 y1 z1)

        On peut calculer t :

        t = - (a*x1 + b*y1 + c*z1 + d)/(a*i + b*j + c*k)

        denominateur = 0 -> pas de solution.
        Cas particuli�ｽ :
        denominateur = 0 et numerateur = 0 et x1 y1 z1 appartient
        au plan (a*x + b*y + c*z + d = 0)
        Alors la droite est contenue dans ce plan
    */
    const float eps    = 0.00001f;
    const Vec p_normal = p.normal;
    const Vec p_org    = p.org;
    const Vec l_dir    = l.dir;
    const Point l_org  = l.org;

    const float denominator = p_normal.dot( l_dir );

    const float a = fabs(denominator);
    if(a < eps) return false;

    float d = -(p_normal.dot(p_org));
    float t = -(p_normal.dot(l_org) + d);
    t /= denominator;

    res = l_org + l_dir * t;

    return true;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST static inline
bool sphere_line(const Sphere& s, const Line& l, Point& res, float& parameter){

    Point center = s.org;
    Point orig   = l.org;
    Vec   dir    = l.dir;

    float a = dir.norm_squared();
    float b = 2.0f * (dir.dot(orig - center));
    float c = (orig - center).norm_squared() - s.radius*s.radius;
    float d = b*b - 4.0f*a*c;

    if (d >= 0){
        d = sqrt(d);

        float t1 = (-b-d) / (2.0f*a);
        float t2 = (-b+d) / (2.0f*a);

        float t  = 1.00001f;
        if (t1 > t2) t = t2;
        else         t = t1;

        res       = orig + dir*t;
        parameter = t;
        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST static inline
bool cylinder_line(const Cylinder& cy, const Line& l, Point& res, float& t){
    // Calculer l'intersection du segment p0 p1 le cylindre est une
    // equation du second degres
    // t�ｽ (v x u)�ｽ + t * 2 * (v x u) * ((o-p) x u) + ((o-p) x u)�ｽ - R�ｽ = 0
    // Avec :
    // v : vecteur directeur du segment
    // u : vecteur directeur du cylindre
    // o : origine du segment
    // p : origine du cylindre
    // t : parametre de l'equation parametrique du segment
    const float eps = 0.0000001f;

    Vec POxU = (l.org - cy.org).cross(cy.dir);
    Vec VxU  = l.dir.cross(cy.dir);

    float a = VxU. dot( VxU  );
    float b = VxU. dot( POxU ) * 2.0f;
    float c = POxU.dot( POxU ) - cy.radius * cy.radius;

    // Determinant de l'equation :
    float d    = b*b - 4.0f*a*c;

    if (d >= 0)
    {
        d = sqrt(d);

        if(a < eps)
            return false;

        float t1 = (-b-d) / (2.0f*a);
        float t2 = (-b+d) / (2.0f*a);
        // On cherche la distance la plus courte :
        t  = 1.00001f;
        if (t1 > t2) t = t2;
        else         t = t1;

        res = l.org + l.dir*t;

        float proj_length = cy.dir.dot( res - cy.org );

        if(proj_length < 0.f || proj_length > cy.length)
            return false;

        return true;
    }
    return false;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST static inline
bool line_line( const Line& line1, const Line& line2, Point& res)
{
    // doctor math :
    // Let's try this with vector algebra. First write the two equations like
    // this.

    // L1 = P1 + a V1
    // L2 = P2 + b V2

    // P1 and P2 are points on each line. V1 and V2 are the direction vectors
    // for each line.

    // If we assume that the lines intersect, we can look for the point on L1
    // that satisfies the equation for L2. This gives us this equation to
    // solve.

    // P1 + a V1 = P2 + b V2

    // Now rewrite it like this.

    // a V1 = (P2 - P1) + b V2

    // Now take the cross product of each side with V2. This will make the
    // term with 'b' drop out.

    // a (V1 X V2) = (P2 - P1) X V2

    // If the lines intersect at a single point, then the resultant vectors
    // on each side of this equation must be parallel, and the left side must
    // not be the zero vector. We should check to make sure that this is
    // true. Once we have checked this, we can solve for 'a' by taking the
    // magnitude of each side and dividing. If the resultant vectors are
    // parallel, but in opposite directions, then 'a' is the negative of the
    // ratio of magnitudes. Once we have 'a' we can go back to the equation
    // for L1 to find the intersection point.

    const float eps = 0.00001f;

    Vec V1xV2   = line1.dir.cross(line2.dir);

    // Denominateur nulle alors pas de solution (droites colineaires)
    float V1xV2norm = V1xV2.norm();
    if(V1xV2norm < eps)
        return false;

    Vec P1P2xV2 = (line2.org - line1.org).cross( line2.dir );

    // Verifie si elle sont contenues dans le meme plan :
    if( (P1P2xV2.cross(V1xV2)).norm() > eps )
        return false;

    float a = P1P2xV2.norm() / V1xV2norm;
    // On determine le signe de 'a' en verifiant que les vecteurs sont ou non
    // de sens opposes :
    if( P1P2xV2.dot(V1xV2) > 0)
        res = line1.org + line1.dir *  a;
    else
        res = line1.org + line1.dir * -a;

    return true;
}

// -----------------------------------------------------------------------------

IF_CUDA_DEVICE_HOST static inline
bool tri_tri( const Triangle& tri1, const Triangle& tri2,
              Point& res1, Point& res2,
              bool& cop, float eps)
{
    cop = false;
    bool s = Inter_tri::tri_tri_intersect_with_isectline(
                (float*)&(tri1.p[0]), (float*)&(tri1.p[1]), (float*)&(tri1.p[2]),
                (float*)&(tri2.p[0]), (float*)&(tri2.p[1]), (float*)&(tri2.p[2]),
                cop,
                (float*)&res1, (float*)&res2,
                eps
                );
    return s;

}

}// END INTER NAMESPACE ========================================================
