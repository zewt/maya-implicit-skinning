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
#ifndef _OGL_WIDGET_SKIN_H_
#define _OGL_WIDGET_SKIN_H_

#include <QEvent>
#include <QKeyEvent>
#include <QTimer>
#include <QTime>

#include "selection_heuristic.hpp"
#include "port_glew.h"
#include "camera.hpp"
#include "cuda_rendering.hpp"

#include "common/msge_stack.hpp"
#include <QtOpenGL/QGLWidget>
#include "common/OGL_widget_enum.hpp"

// FORWARD DEFS ----------------------------------------------------------------
// Can't include the header because of interdependencies between IO_interface
// and OGLWidget classes
class IO_interface_skin;
class Main_window_skin;
// END FORWARD DEFS ------------------------------------------------------------

class OGL_widget_skin : public QGLWidget {
    Q_OBJECT
public:

    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    OGL_widget_skin(QWidget *parent, QGLWidget* sh, Main_window_skin* m);
    OGL_widget_skin(QWidget *parent, Main_window_skin* m);
    ~OGL_widget_skin();

    /// update the pivot position given the current mode of rotation
    void update_pivot();

    // -------------------------------------------------------------------------
    /// @name Static tools
    // -------------------------------------------------------------------------

    /// initialize the glew to access openGL extensions must be done once per
    /// openGL context
    static void init_glew();

    // -------------------------------------------------------------------------
    /// @name Getter & Setters
    // -------------------------------------------------------------------------

    /// Draw the skeleton or the graph
    void set_draw_skeleton(bool s){ _render_ctx->_skeleton = s; }

    /// Enable/disable phong rendering
    void set_phong(bool s){ _render_ctx->_plain_phong = s; }

    /// Enable/disable textures in phong rendering
    void set_textures(bool s){ _render_ctx->_textures = s; }

    /// Enable/disable raytracing
    void set_raytracing(bool s){ _render_ctx->_raytrace = s; }

    /// Draw the mesh in rest position or in animated position
    void set_rest_pose(bool s){ _render_ctx->_rest_pose = s; }

    bool rest_pose(){ return _render_ctx->_rest_pose; }

    void set_draw_mesh(bool s){ _render_ctx->_draw_mesh = s; }

    bool draw_mesh(){ return _render_ctx->_draw_mesh; }

    /// @return transclucent or plain phong rendering ?
    bool phong_rendering() const { return _render_ctx->_plain_phong; }

    /// @return true if raytracing enable
    bool raytrace() const { return _render_ctx->_raytrace;    }

    Select_type<int>* get_heuristic(){ return _heuristic; }

    Camera* camera(){ return &_cam; }

    Vec3_cu pivot() const { return _pivot; }

    EOGL_widget::Pivot_t pivot_mode() const { return _pivot_mode; }

    void set_pivot_user(const Vec3_cu& v){ _pivot_user = v; }

    void set_pivot_mode(EOGL_widget::Pivot_t m){ _pivot_mode = m; }

    void set_main_window(Main_window_skin* m);
    Main_window_skin* get_main_window();

    // -------------------------------------------------------------------------
    /// @name Public attributes
    // -------------------------------------------------------------------------

    /// use to draw temporary message on screen
    Msge_stack* _msge_stack;

    /// wether the camera tracks the pivot point or not
    bool _track_pivot;

signals:
    // -------------------------------------------------------------------------
    /// @name Signals
    // -------------------------------------------------------------------------

    /// Emited on mouse press events
    void clicked();
    /// Emited for each frame (only if (g_save_anim || g_shooting_state))
    void drawing();

public slots:
    // -------------------------------------------------------------------------
    /// @name Slots
    // -------------------------------------------------------------------------

    /// Change the input/output handler to another behavior
    void set_io(EOGL_widget::IO_t io_type);

    /// set the selection mode
    void set_selection(EOGL_widget::Select_t select_mode);

 protected:
    // -------------------------------------------------------------------------
    /// @name Events
    // -------------------------------------------------------------------------

    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();
    void mousePressEvent  ( QMouseEvent* event );
    void mouseReleaseEvent( QMouseEvent* event );
    void wheelEvent       ( QWheelEvent* event );
    void mouseMoveEvent   ( QMouseEvent* event );
    void keyPressEvent    ( QKeyEvent*   event );
    void keyReleaseEvent  ( QKeyEvent*   event );
    void enterEvent       ( QEvent*      event );
    void leaveEvent       ( QEvent*      event );

    /// Hack to generate a key event when tab is stroke.
    /// By default Qt receive the event but eat it in order to switch the focus
    /// from widget to widget. We bypass this behavior by overriding the event()
    /// method. This enables us to forward the tab event to keyPressEvent() or
    /// keyReleaseEvent()
    bool event( QEvent* event );

private:

    // -------------------------------------------------------------------------
    /// @name Internal tools
    // -------------------------------------------------------------------------

    /// Factorize the initialization of some attributes
    void init();

    /// if necessary updates camera position when tracking enabled
    void update_camera();

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    /// handle mouse and keyboards according to the desire modeling mode
    IO_interface_skin* _io;

    /// The main window this viewport belongs to.
    Main_window_skin* _main_win;

    /// Context for the rendering in cuda
    Render_context_cu* _render_ctx;

    /// Camera attached to the viewport
    Camera _cam;

    /// Pivot point position. Used by the camera to rotate around it.
    Vec3_cu _pivot;

    /// Pivot point defined by the user
    Vec3_cu _pivot_user;

    /// mode of rotation: defines what's the standard behavior to compute
    /// automatically the pivot point
    EOGL_widget::Pivot_t _pivot_mode;

    /// Current heuristic for mesh's points selection. Which defines the
    /// selection area (point, square, circle etc.) used to select the mesh's
    /// points.
    Select_type<int>* _heuristic;

    /// use to redraw screen at regular intervalles
    QTimer* _refresh_screen_timer;

    bool _is_mouse_in;   ///< true when mouse cursor hoover the viewport

    float _mouse_x;
    float _mouse_y;
};

#endif // _OGL_WIDGET_H_
