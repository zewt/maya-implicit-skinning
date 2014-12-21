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

#include "port_glew.h"

#include <QGLWidget>

#include "main_window_skin.hpp"


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

    QGLWidget* shared_viewport(){ return _hidden; }

private slots:

signals:
    void active_viewport_changed(int id);
    /// Update status bar
    void update_status(QString);

private:
    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
    bool _skel_mode;

    /// opengl shared context between all viewports
    /// (in order to share VBO textures etc.)
    OGL_widget_skin_hidden* _hidden;
};

#endif // OGL_VIEWPORTS_HPP__
