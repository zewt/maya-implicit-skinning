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
#ifndef OGL_VIEWPORTS_SKIN_HPP__
#define OGL_VIEWPORTS_SKIN_HPP__

#include <QFrame>
#include <vector>
#include <QLayout>

#include "main_window_skin.hpp"
#include "OGL_widget_skin.hpp"


class Viewport_frame_skin;

typedef  std::vector<OGL_widget_skin*> Vec_viewports;

// =============================================================================
class OGL_widget_skin_hidden : public QGLWidget {
public:
    OGL_widget_skin_hidden(QWidget* w) : QGLWidget(w) {
    }

    void initializeGL();
};
// =============================================================================

/** @class OGL_viewports
  @brief Multiple viewports handling with different layouts

*/
class OGL_viewports_skin : public QFrame {
    Q_OBJECT
public:
    OGL_viewports_skin(QWidget* w, Main_window_skin* m);

    ~OGL_viewports_skin();

    /// Updates all viewports
    void updateGL();

    // -------------------------------------------------------------------------
    /// @name Getter & Setters
    // -------------------------------------------------------------------------
    enum Layout_e { SINGLE, VDOUBLE, HDOUBLE, FOUR };

    /// Erase all viewports and rebuild them according to the specified layout
    /// 'setting'
    void set_viewports_layout(Layout_e setting);

    /// @warning the list is undefined if used after a call to
    /// set_viewports_layout()
    Vec_viewports& get_viewports();

    /// sets pivot for all viewports
    void set_pivot_mode(EOGL_widget::Pivot_t m);

    /// @return the active frame
    OGL_widget_skin* active_viewport(){ return _current_viewport; }

    QGLWidget* shared_viewport(){ return _hidden; }

    // -------------------------------------------------------------------------
    /// @name Events
    // -------------------------------------------------------------------------
    void enterEvent( QEvent* e);

    // -------------------------------------------------------------------------
    /// @name Qt Signals & Slots
    // -------------------------------------------------------------------------
private slots:
    /// Designed to be called each time a single viewport draws one frame.
    void incr_frame_count();
    void active_viewport_slot(int id);

signals:
    void frame_count_changed(int);
    void active_viewport_changed(int id);
    /// Update status bar
    void update_status(QString);

private:
    // -------------------------------------------------------------------------
    /// @name Tools
    // -------------------------------------------------------------------------
    QLayout* gen_single ();
    QLayout* gen_vdouble();
    QLayout* gen_hdouble();
    QLayout* gen_four   ();

    /// suppress all viewports and layouts
    void erase_viewports();

    /// Creates a new viewport with the correct signals slots connections
    OGL_widget_skin* new_viewport(Viewport_frame_skin* ogl_frame);

    /// Creates a new viewport frame with the correct signals slots connections
    Viewport_frame_skin* new_viewport_frame(QWidget* parent, int id);

    /// Sets the frame color by replacing its styleSheet color
    void set_frame_border_color(Viewport_frame_skin* f, int r, int g, int b);

    void first_viewport_as_active();

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
    bool _skel_mode;

    /// Vector of OGL_widget
    Vec_viewports _viewports;

    /// List of frames associated to the viewports
    std::vector<Viewport_frame_skin*> _viewports_frame;

    /// The active viewport
    OGL_widget_skin* _current_viewport;

    /// opengl shared context between all viewports
    /// (in order to share VBO textures etc.)
    OGL_widget_skin_hidden* _hidden;

    /// Layout containing all viewports
    QLayout* _main_layout;

    /// main widow the widget's belongs to
    Main_window_skin* _main_window;

    /// sum of frames drawn by the viewports
    int _frame_count;

    /// Fps counting timer
    QTime  _fps_timer;
};
// =============================================================================

class Viewport_frame_skin : public QFrame {
    Q_OBJECT
public:

    Viewport_frame_skin(QWidget* w, int id) : QFrame(w), _id(id)
    {
        setFrameShape(QFrame::Box);
        setFrameShadow(QFrame::Plain);
        setLineWidth(1);
        setStyleSheet(QString::fromUtf8("color: rgb(0, 0, 0);"));
    }


    int id() const { return _id; }

signals:
    void active(int);

private slots:
    void activate(){
        emit active(_id);
    }

private:
    int _id;
};
// =============================================================================

#endif // OGL_VIEWPORTS_HPP__
