#ifndef IBL_TOOLS_HPP__
#define IBL_TOOLS_HPP__


// =============================================================================
namespace IBL {
// =============================================================================

/// Compute numerically the inverse of f within the range [xmin, xmax].
/// f must be monotonic
/// @param y : abscisa of f_inv
/// @param f : function to invert (f must be monotonic)
/// @param xmin, xmax : abscisa range used to perform the dychotomic search to
/// find y.
/// @param eps : threshold of precision to stop the dychotomic search
/// @return the value returned by f_inv( y ) (or the x corresponding to f(x) = y)
/// y must in [f(min), f(max)] otherwise result is undefined.
double f_inverse(double y, double (*f)(double x), double xmin, double xmax, double eps = 1e-5);

}// END IBL ====================================================================

#endif // IBL_TOOLS_HPP__
