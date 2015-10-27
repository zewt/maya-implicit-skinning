#ifndef IBL_CONTROLLER_HPP__
#define IBL_CONTROLLER_HPP__

#include <string.h>
#include "structs.hpp"

// =============================================================================
namespace IBL {
// =============================================================================

class Ctrl_setup {
private:
    IBL::float2 _pt[3];     ///< the control points of the curve from left to right
    float       _slopes[2]; ///< Slopes between pt[0]-pt[1] and pt[1]-pt[2] respectively
public:
    Ctrl_setup()
    {
        _pt[0].x = _pt[1].x = _pt[2].x = 0.f;
        _pt[0].y = _pt[1].y = _pt[2].y = 0.f;
        _slopes[0] = _slopes[1] = -1.f;
    }

    Ctrl_setup(IBL::float2 p0, IBL::float2 p1, IBL::float2 p2, float s0, float s1)
    {
        // TODO: clamp values maybe ? or assert ?
        _pt[0] = p0; _pt[1] = p1; _pt[2] = p2;
        _slopes[0] = s0; _slopes[1] = s1;
    }

    bool operator==(const Ctrl_setup &rhs) const {
        return !memcmp(_pt, rhs._pt, sizeof(_pt)) &&
            !memcmp(_slopes, rhs._slopes, sizeof(_slopes));
    }

    IBL::float2 p0() const { return _pt[0];     }
    IBL::float2 p1() const { return _pt[1];     }
    IBL::float2 p2() const { return _pt[2];     }
    float       s0() const { return _slopes[0]; }
    float       s1() const { return _slopes[1]; }

    void p0(IBL::float2 v) {
        _pt[0].x = clamp(v.x, -3.f, p1().x);
        _pt[0].y = clamp(v.y, 0.f, 1.f);
    }
    void p1(IBL::float2 v) {
        _pt[1].x = clamp(v.x, p0().x, p2().x);
        _pt[1].y = clamp(v.y, 0.f, 1.f);
    }
    void p2(IBL::float2 v) {
        _pt[2].x = clamp(v.x, p1().x, 3.f);
        _pt[2].y = clamp(v.y, 0.f, 1.f);
    }
    void s0(float v) { _slopes[0] = clamp(v, -4.f, 12.f); }
    void s1(float v) { _slopes[1] = clamp(v, -4.f, 12.f); }
};

    // =========================================================================
    namespace Shape {
    // =========================================================================

    // TODO: Ctrl_setup organic();
    IBL::Ctrl_setup caml();
    IBL::Ctrl_setup finger();
    IBL::Ctrl_setup elbow();
    IBL::Ctrl_setup flat_up();
    IBL::Ctrl_setup flat_down();

    }
    // END Shape ===============================================================

    class Ctr_base {
    public:
        /// @param gradient_dot is cos(theta) where theta is the angle between
        /// two gradient usually. The dot product between two normalized
        /// gradient conviniently gives cos(theta) thus the name gradient_dot
        virtual float eval( float gradient_dot ) const = 0;
    };

    // =========================================================================
    namespace Discreet {
    // =========================================================================

    class Controller : public Ctr_base {
    public:
        Controller(const Ctrl_setup& shape, int nb_samples);
        ~Controller();

        /// @param gradient_dot is cos(teta) where teta is the angle between
        /// two gradient usually. The dot product between two normalized
        /// gradient conviniently gives cos(teta) thus the name gradient_dot
        float eval( float gradient_dot ) const;

        void update_shape(const Ctrl_setup& shape, int nb_samples);

    private:
        Ctrl_setup   _shape;
        IBL::float2* _func_vals;
    };

    }
    // END Discreet ============================================================


    // =========================================================================
    namespace Continuous {
    // =========================================================================

    class Controller : public Ctr_base {
    public:
        Controller(const Ctrl_setup& shape) : _shape(shape)
        { }

        float eval( float gradient_dot ) const;

        void update_shape(const Ctrl_setup& shape);

    private:
        Ctrl_setup   _shape;
    };

    }
    // END Continuous ==========================================================

}
// END IBL =====================================================================

#endif // IBL_CONTROLLER_HPP__
