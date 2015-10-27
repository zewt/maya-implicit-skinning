#include "opening.hpp"

#include "funcs.hpp"

namespace IBL {

/// Compute numerically the inverse of f within the range [xmin, xmax].
/// f must be monotonic
/// @param y : abscisa of f_inv
/// @param f : function to invert (f must be monotonic)
/// @param xmin, xmax : abscisa range used to perform the dychotomic search to
/// find y.
/// @param eps : threshold of precision to stop the dychotomic search
/// @return the value returned by f_inv( y ) (or the x corresponding to f(x) = y)
/// y must in [f(min), f(max)] otherwise result is undefined.
double f_inverse(double y,
                 double (*f)(double x),
                 double xmin,
                 double xmax,
                 double eps = 1e-5)
{
    // Check we are within range [xmin, xmax]
    assert( std::min( std::max( f(xmin), f(xmax) ), y ) == y );

    bool rising = f(xmax) > f(xmin);
    double x0 = xmin;
    double x1 = xmax;

    double x_mid = x0;
    int acc = 0;
    while( std::abs(f(x_mid) - y) > eps )
    {
        if( acc++ > 1000) break; // avoid infinite loops

        x_mid = (x0+x1) * 0.5f;
//        const bool s = rising ? f(x_mid) > y : f(x_mid) <= y;
//        if( s ) x1 = x_mid;

        if ( !( (f(x_mid) > y) ^ (rising)) ) x1 = x_mid;
        else                                 x0 = x_mid;
    }

    return x_mid;
}

// =============================================================================
namespace Opening {
// =============================================================================

Base* make(Kind_t opening_type)
{
    typedef IBL::Opening::Discreet_hyperbola Dh;

    Base* opening = 0;

    switch (opening_type)
    {
    case IBL::Opening::LINE:        opening = new IBL::Opening::Line();    break;
    case IBL::Opening::DIAMOND:     opening = new IBL::Opening::Diamond(); break;
    case IBL::Opening::OPEN_TANH:   opening = new Dh(Dh::OPEN_TANH);       break;
    case IBL::Opening::CLOSED_H:    opening = new Dh(Dh::CLOSED_HERMITE);  break;
    case IBL::Opening::CLOSED_TANH: opening = new Dh(Dh::CLOSED_TANH);     break;
    default:
        std::cerr << "ERROR: openig type doesn't exists" << std::endl;
        break;
    }

    return opening;
}

// -----------------------------------------------------------------------------

bool Pan_hf::_is_init;
float Pan_hf::_vals[];

// -----------------------------------------------------------------------------

void Pan_hf::init_samples()
{
    if(_is_init) return;


    for(int i = 0; i < _nb_samples; i++)
    {
        float t = 2.f * ((float)i / (float)(_nb_samples-1));
        _vals[i] = (float)f_inverse(t, f_hyperbola, 0.f, 1.f);
        //printf("%d- %f; ",i, _vals[i]);
    }
    _vals[_nb_samples-1] = 1.f;
    _is_init = true;
}

// -----------------------------------------------------------------------------


}// END IBL ====================================================================

}// END OPENING ================================================================

