#ifndef OPERATORS_CTRL_HPP__
#define OPERATORS_CTRL_HPP__

#include "blending_lib/controller.hpp"

class Operators_ctrl {
public:

    Operators_ctrl():
        _disp_size(240),
        _display_operator(false),
        _display_controller(false)
    { }


    /// @warning very slow !
    void update_bulge();


    void update_displayed_operator_texture();

    void set_bulge_magnitude(float magnitude);
    void set_ricci_n(float N);

    void set_global_controller(const IBL::Ctrl_setup& shape);
    void set_controller(int ctrl_id, const IBL::Ctrl_setup& shape);

    void print_controller();

    void set_display_size(int s){
        _disp_size = s;
        update_displayed_operator_texture();
    }

    IBL::Ctrl_setup get_global_controller();

    void set_display_operator  (bool state){ _display_operator = state;   }
    void set_display_controller(bool state){ _display_controller = state; }
    bool get_display_operator  (          ){ return _display_operator;    }
    bool get_display_controller(          ){ return _display_controller;  }
    int  get_display_size      (          ){ return _disp_size;           }


    /// @name Parameters for n-ary operators
    /// @{
    /// TODO: The mystics values of these should be explained
    /// TODO: an enum field would be more appropriate, n_ary.hpp should be
    /// define them and provide an accessor set(the_enum) to set them.
    void set_ricci_operator_n( float value );
    void set_deform_operator_wA0A0( float value );
    void set_deform_operator_wA0A1( float value );
    void set_deform_operator_wA1A1( float value );
    void set_deform_operator_wA1A0( float value );
    void set_contact_a0_1( float value );
    void set_contact_w1( float value );
    void set_contact_gji( float value );
    void set_contact_a0_2( float value );
    void set_contact_w2( float value );
    void set_contact_gij( float value );
    /// @}

private:
    int  _disp_size; ///< size of the blending operator when displayed
    bool _display_operator;
    bool _display_controller;
};

#endif // OPERATORS_CTRL_HPP__
