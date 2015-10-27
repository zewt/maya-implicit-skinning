#ifndef IBL_FUNCS_HPP__
#define IBL_FUNCS_HPP__

#include <cmath>

/**
 * @file funcs.hpp
 * @brief various cartesian functions to be shared in IBL
 */
// =============================================================================
namespace IBL {
// =============================================================================

static inline double square(double t){
    return t * t;
}

// OPENING HYPERBOlA FUNCS -----------------------------------------------------

static inline double r(double t){
    return log( 1. + 1. / (exp(t) - 1.) );
}

static inline double s(double t){
    return log( 1. + r( exp(t) - 1.) );
}

static inline double u0(double t){
    return s( t * exp(1.) ) / exp(1.);
}

/// monotonically increasing function f(-inf)=-inf / f(0) = 0 / f(1) = +inf
static inline double f_hyperbola(double t)
{
    const double res = u0(1. - t) + t;
    return res > 2. ? 2. : res;
}

// END OPENING HYPERBOLA FUNCS -------------------------------------------------

// PROFILE HYPERBOLA FUNCS -----------------------------------------------------

// Cartesian hyperbola curve profile
// 1 - (ln(1 + ln(1 + 1/(exp(exp((1-t) * exp(1))-1)-1))) / exp(1))
static inline double u(double t){
    return 1. - u0(1.-t);
}

static inline double dr(double t){
    double et = exp(t);
    double dg = et * (-1. / square(et-1.));
    double dln = 1. / (1.  + 1./(et - 1.));
    return dg*dln;
}

static inline double ds(double t){
    double et = exp(t);
    double dg =  et * dr(et-1.);
    double dln = 1. / (1. + r(et-1.));
    return dg*dln;
}

static inline double du0(double t){
    return ds(t * exp(1.));
}

// First derivative of u()
static inline double du(double t){
    return du0(1.-t);
}

// END PROFILE HYPERBOLA FUNCS -------------------------------------------------

}// END IBL ====================================================================

#endif // IBL_FUNCS_HPP__
