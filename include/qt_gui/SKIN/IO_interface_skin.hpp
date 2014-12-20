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
      _movement_speed(1.f),
      _rot_speed(0.01f),
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
    }

    // -------------------------------------------------------------------------

    void right_view(){}
    void left_view(){}
    void top_view(){}
    void bottom_view(){}
    void front_view(){}
    void rear_view(){}

    // -------------------------------------------------------------------------

    IBL::float2 increase(IBL::float2 val, float incr, bool t)
    {
        if(t) val.x += incr;
        else  val.y += incr;
        return val;
    }

    void mousePressEvent  (QMouseEvent* e){ e->ignore(); }
    void mouseReleaseEvent(QMouseEvent* e){ e->ignore(); }
    void mouseMoveEvent   (QMouseEvent* e){ e->ignore(); }
    void wheelEvent       (QWheelEvent* e){ e->ignore(); }
    void keyPressEvent    (QKeyEvent*   e){ e->ignore(); }
    void keyReleaseEvent  (QKeyEvent*   e){ e->ignore(); }

    /// Update the gizmo orientation this is to be called by OGL_widget
    /// every frame
    virtual void update_frame_gizmo(){ }

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

    float _movement_speed;  ///< speed of the camera movements
    float _rot_speed;       ///< rotation speed of the camera

    /// @name Opengl matrix
    /// which state matches the last call of update_gl_matrix()
    /// @{
    GLint    _viewport  [4];
    GLdouble _modelview [16];
    GLdouble _projection[16];
    /// @}

    OGL_widget_skin*  _gl_widget;
    Camera*           _cam;
    Main_window_skin* _main_win;
};

#endif // IO_INTERFACE_HPP__
