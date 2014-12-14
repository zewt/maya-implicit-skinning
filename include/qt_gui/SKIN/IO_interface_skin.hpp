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
#ifndef IO_INTERFACE_SKIN_HPP__
#define IO_INTERFACE_SKIN_HPP__

/** @brief Abstract class handling mouse and keyboards events

    This class is design to provide dynamic behavior of mouse and keys.
    You can find specialised behaviors in IO_xxx class family.
*/

#include <QMouseEvent>
#include "SKIN/main_window_skin.hpp"
#include "port_glew.h"
#include "cuda_ctrl.hpp"
#include "SKIN/OGL_widget_skin.hpp"
#include "intersection.hpp"

#ifndef M_PI
#define M_PI (3.14159265358979323846f)
#endif

/////////////////////////////////////////////////////////////
// TODO: delete all this and provide proper header interface via cuda_ctrl

// From cuda_globals.hpp -------
#include "skeleton.hpp"
extern Skeleton* g_skel;

#include "animesh.hpp"
#include "animesh_enum.hpp"
extern Animesh* g_animesh;
// -------

extern bool g_shooting_state;
extern bool g_save_anim;
#include "camera.hpp"
/////////////////////////////////////////////////////////////

class IO_interface_skin {
public:
    IO_interface_skin(OGL_widget_skin* gl_widget) :
      _is_ctrl_pushed(false),
      _is_alt_pushed(false),
      _is_tab_pushed(false),
      _is_maj_pushed(false),
      _is_space_pushed(false),
      _is_right_pushed(false),
      _is_left_pushed(false),
      _is_mid_pushed(false),
      _is_gizmo_grabed(false),
      _movement_speed(1.f),
      _rot_speed(0.01f),
      _gizmo_tr(),
      _gl_widget(gl_widget),
      _cam(gl_widget->camera()),
      _main_win(gl_widget->get_main_window())
    {  }

    virtual ~IO_interface_skin(){}

    // -------------------------------------------------------------------------

    void update_gl_matrix(){
        glGetIntegerv(GL_VIEWPORT, _viewport);
        glGetDoublev(GL_MODELVIEW_MATRIX,  _modelview);
        glGetDoublev(GL_PROJECTION_MATRIX, _projection);
    }

    // -------------------------------------------------------------------------

    /// Draw a message on the viewport
    void push_msge(const QString& str){
        _gl_widget->_msge_stack->push(str, true);
    }

    // -------------------------------------------------------------------------

    virtual void mousePressEvent(QMouseEvent* event){
        const int x = event->x();
        const int y = event->y();

        _old_x = x;
        _old_y = _cam->height() - y;

        _cam_old_x = x;
        _cam_old_y = _cam->height() - y;

        _is_right_pushed = (event->button() == Qt::RightButton);
        _is_left_pushed  = (event->button() == Qt::LeftButton);
        _is_mid_pushed   = (event->button() == Qt::MidButton);

        if( _gl_widget->_draw_gizmo && _is_left_pushed )
            _is_gizmo_grabed = gizmo()->select_constraint(*_cam, x, y);
    }

    // -------------------------------------------------------------------------

    virtual void mouseReleaseEvent(QMouseEvent* event){
        _is_right_pushed = (event->button() == Qt::RightButton) ? false : _is_right_pushed;
        _is_left_pushed  = (event->button() == Qt::LeftButton)  ? false : _is_left_pushed;
        _is_mid_pushed   = (event->button() == Qt::MidButton)   ? false : _is_mid_pushed;

        if(!_is_left_pushed){
            gizmo()->reset_constraint();
            _is_gizmo_grabed = false;
        }
    }

    // -------------------------------------------------------------------------

    /// Camera rotation
    virtual void mouseMoveEvent(QMouseEvent* event){
        using namespace Cuda_ctrl;
        const int x = event->x();
        const int y = event->y();

        _gizmo_tr = gizmo()->slide(*_cam, x, y);

        if(_is_right_pushed)
        {
            int dx = x - _cam_old_x;
            int dy = _cam->height() - y - _cam_old_y;
            //rotation around x axis (to manage the vertical angle of vision)
            const float pitch = dy * _rot_speed / M_PI;
            //rotation around the y axis (vertical)
            const float yaw = -dx * _rot_speed / M_PI;
            //no roll
            _cam->update_dir(yaw, pitch, 0.f);

            if( _gl_widget->pivot_mode() != EOGL_widget::FREE )
            {
                // rotate around the pivot
                _gl_widget->update_pivot();
                Vec3_cu v = _gl_widget->pivot();

                Vec3_cu tcam = _cam->get_pos();
                float dx = v.x - tcam.x;
                float dy = v.y - tcam.y;
                float dz = v.z - tcam.z;
                float d = sqrtf(dx*dx + dy*dy + dz*dz);
                Vec3_cu dir = _cam->get_dir();
                _cam->set_pos(v.x - dir.x * d,
                              v.y - dir.y * d,
                              v.z - dir.z * d );
            }
        }

        // camera straff
        if( _is_mid_pushed && _is_maj_pushed )
        {
            int dx = x;
            int dy = (_cam->height() - y);
            Point_cu p     = _cam->un_project( Point_cu(        dx,         dy, 0.f) );
            Point_cu old_p = _cam->un_project( Point_cu(_cam_old_x, _cam_old_y, 0.f) );
            Vec3_cu v = old_p - p;

            if( _cam->is_ortho() )
            {

            }
            else
            {/*
                Inter::Plane p( -_cam->get_dir(), _gl_widget->pivot());
                Ray_cu r = _cam->cast_ray(dx, dy);
                Inter::Line l(r.dir, r.pos);
                Vec3_cu res;
                Inter::plane_line(p, l, res);

                v = _gl_widget->pivot() - res;
                */
                //v.y *= -1.f;
            }

            Vec3_cu npos = _cam->get_pos() + v;
            _cam->set_pos( npos.x, npos.y, npos.z );
            _gl_widget->set_pivot_user(  _gl_widget->pivot() + v );
        }

        _cam_old_x = x;
        _cam_old_y = _cam->height() - y;
    }

    // -------------------------------------------------------------------------

    /// Camera movements back and forth
    virtual void wheelEvent(QWheelEvent* event){
        float numDegrees = event->delta() / 8.;
        float numSteps = numDegrees / 15.;

        if(event->buttons() == Qt::NoButton )
        {
            float sign  = numSteps > 0 ? 1.f : -1.f;
            float width = _cam->get_frustum_width();
            float new_width = std::max(width - sign * _movement_speed, 0.3f);

            if(_cam->is_ortho())
                _cam->set_frustum_width(new_width);
            else
                _cam->update_pos(Vec3_cu(0.f, 0.f,  _movement_speed*1.f*sign));
        }
    }

    // -------------------------------------------------------------------------

    void right_view(){
        _cam->set_dir_and_up(-1.f, 0.f, 0.f,
                             0.f, 1.f, 0.f);

        Vec3_cu p = _cam->get_pos();
        Vec3_cu v = _gl_widget->pivot();
        _cam->set_pos( v.x + (p-v).norm(), v.y, v.z);
    }

    void left_view(){
        _cam->set_dir_and_up(1.f, 0.f, 0.f,
                             0.f, 1.f, 0.f);

        Vec3_cu p = _cam->get_pos();
        Vec3_cu v = _gl_widget->pivot();
        _cam->set_pos( v.x - (p-v).norm(), v.y, v.z);
    }

    void top_view(){
        _cam->set_dir_and_up(0.f, -1.f, 0.f,
                             0.f, 0.f, -1.f);

        Vec3_cu p = _cam->get_pos();
        Vec3_cu v = _gl_widget->pivot();
        _cam->set_pos( v.x, v.y + (p-v).norm(), v.z);
    }

    void bottom_view(){
        _cam->set_dir_and_up(0.f, 1.f, 0.f,
                             0.f, 0.f, 1.f);

        Vec3_cu p = _cam->get_pos();
        Vec3_cu v = _gl_widget->pivot();
        _cam->set_pos( v.x, v.y - (p-v).norm(), v.z);
    }

    void front_view(){
        _cam->set_dir_and_up(0.f, 0.f, -1.f,
                             0.f, 1.f, 0.f);

        Vec3_cu p = _cam->get_pos();
        Vec3_cu v = _gl_widget->pivot();
        _cam->set_pos( v.x, v.y, v.z + (p-v).norm());
    }

    void rear_view(){
        _cam->set_dir_and_up(0.f, 0.f, 1.f,
                             0.f, 1.f, 0.f);

        Vec3_cu p = _cam->get_pos();
        Vec3_cu v = _gl_widget->pivot();
        _cam->set_pos( v.x, v.y, v.z - (p-v).norm());
    }

    // -------------------------------------------------------------------------

    IBL::float2 increase(IBL::float2 val, float incr, bool t)
    {
        if(t) val.x += incr;
        else  val.y += incr;
        return val;
    }

    // -------------------------------------------------------------------------

    virtual void keyPressEvent(QKeyEvent* event){
        using namespace Cuda_ctrl;

        // Check special keys
        switch( event->key() ){
        case Qt::Key_Control: _is_ctrl_pushed = true; break;
        case Qt::Key_Alt:     _is_alt_pushed  = true; break;
        case Qt::Key_Tab:     _is_tab_pushed  = true; break;
        case Qt::Key_Shift:   _is_maj_pushed  = true; break;
        case Qt::Key_Space:   _is_space_pushed= true; break;
        case Qt::Key_Up :
            _cam->update_pos(Vec3_cu(0.f, 0.f,  2.*_movement_speed*1.f));
            break;
        case Qt::Key_Down :
            _cam->update_pos(Vec3_cu(0.f, 0.f, -2.*_movement_speed* 1.f));
            break;
        case Qt::Key_Left : //screenshot();
            break;
        }

        const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

        IBL::Ctrl_setup shape;
        if( set.size() > 0 )
            shape = Cuda_ctrl::_skeleton.get_joint_controller(set[set.size()-1]);
        else
            shape = Blending_env::get_global_ctrl_shape();


        // Check standard characters :
        QString t = event->text();
        QChar c = t[0];
        using namespace Constants;
        bool view_changed = true;
        switch (c.toLatin1()) {
        // ''''''''''''''''''''''''''''''''''''''''''''
        // Change operator parameters :

        // Change abscissa :
        case 'E': shape.p0( increase(shape.p0(),  0.1f, true) );break;
        case 'e': shape.p0( increase(shape.p0(), -0.1f, true) );break;
        case 'Z': shape.p1( increase(shape.p1(),  0.1f, true) );break;
        case 'z': shape.p1( increase(shape.p1(), -0.1f, true) );break;
        case 'A': shape.p2( increase(shape.p2(),  0.1f, true) );break;
        case 'a': shape.p2( increase(shape.p2(), -0.1f, true) );break;
        // Change ordinate :
        case 'D': shape.p0( increase(shape.p0(),  0.02f, false) );break;
        case 'd': shape.p0( increase(shape.p0(), -0.02f, false) );break;
        case 'S': shape.p1( increase(shape.p1(),  0.02f, false) );break;
        case 's': shape.p1( increase(shape.p1(), -0.02f, false) );break;
        case 'Q': shape.p2( increase(shape.p2(),  0.02f, false) );break;
        case 'q': shape.p2( increase(shape.p2(), -0.02f, false) );break;
        // Change Stifness
        case 'X': shape.s0( shape.s0() +  0.2f );break;
        case 'x': shape.s0( shape.s0() + -0.2f );break;
        case 'W': shape.s1( shape.s1() +  0.2f );break;
        case 'w': shape.s1( shape.s1() + -0.2f );break;
        // ''''''''''''''''''''''''''''''''''''''''''''

        // OTHER STUFF
        case 'u' : _anim_mesh->update_base_potential(); push_msge("update base potential"); break;
        // Camera angles :
        case '8': top_view(); push_msge("top");       break;
        case '2': bottom_view(); push_msge("bottom"); break;
        case '6': right_view(); push_msge("right");   break;
        case '4': left_view(); push_msge("left");     break;
        case '1': front_view(); push_msge("front");   break;
        case '3': rear_view(); push_msge("rear");     break;
        case '5': _cam->set_ortho(!_cam->is_ortho()); break;

        default: view_changed = false; break;
        }
        // This updates the raytracing drawing and sets back _raytrace_again to false
        Cuda_ctrl::_display._raytrace_again = (view_changed || Cuda_ctrl::_display._raytrace_again);

        // Update the controler function
        if(set.size() > 0)
            Cuda_ctrl::_skeleton.set_joint_controller(set[set.size()-1], shape);
        else
            Blending_env::set_global_ctrl_shape(shape);

        _main_win->update_ctrl_spin_boxes(shape);

        Cuda_ctrl::_operators.update_displayed_operator_texture();

        // Ok this is a bit hacky but it enables main window to see keypress events
        // and handle them. Be aware that a conflict will appear if the same shortcut
        // is used here and inside MainWindow.
        event->ignore();
    }

    // -------------------------------------------------------------------------

    virtual void keyReleaseEvent(QKeyEvent* event){
        // Check special keys
        switch( event->key() ){
        case Qt::Key_Control: _is_ctrl_pushed = false; break;
        case Qt::Key_Alt:     _is_alt_pushed  = false; break;
        case Qt::Key_Tab:     _is_tab_pushed  = false; break;
        case Qt::Key_Shift:   _is_maj_pushed  = false; break;
        case Qt::Key_Space:   _is_space_pushed= false; break;
        }
    }

    /// Update the gizmo orientation this is to be called by OGL_widget
    /// every frame
    virtual void update_frame_gizmo(){ }

    /// Shortcut function to get the gizmo
    Gizmo* gizmo(){ return _gl_widget->gizmo(); }

    /// Shortcut to get the skeleton's kinematic
    Kinematic* kinec(){ return g_skel->_kinec; }


    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
    float _old_x;           ///< last mouse click in pixel x
    float _old_y;           ///< last mouse click in pixel y

    float _cam_old_x;
    float _cam_old_y;

    bool _is_ctrl_pushed;   ///< is control key pushed
    bool _is_alt_pushed;    ///< is alt key pushed
    bool _is_tab_pushed;    ///< is tabulation key pushed
    bool _is_maj_pushed;    ///< is shift key pushed
    bool _is_space_pushed;  ///< is space key pushed
    bool _is_right_pushed;  ///< is mouse right button pushed
    bool _is_left_pushed;   ///< is mouse left button pushed
    bool _is_mid_pushed;    ///< is mouse middle
    bool _is_gizmo_grabed;  ///< is a gizmo constraint selected

    float _movement_speed;  ///< speed of the camera movements
    float _rot_speed;       ///< rotation speed of the camera

    /// @name Opengl matrix
    /// which state matches the last call of update_gl_matrix()
    /// @{
    GLint    _viewport  [4];
    GLdouble _modelview [16];
    GLdouble _projection[16];
    /// @}

    /// Gizmo transformation when grabed
    TRS _gizmo_tr;

    OGL_widget_skin*  _gl_widget;
    Camera*           _cam;
    Main_window_skin* _main_win;
};

#endif // IO_INTERFACE_HPP__
