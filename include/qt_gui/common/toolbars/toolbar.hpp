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
#ifndef TOOLBAR_HPP__
#define TOOLBAR_HPP__

// -----------------------------------------------------------------------------

#include "qt_gui/common/toolbars/widget_fitting.hpp"
#include "qt_gui/common/toolbars/widget_selection.hpp"
#include "qt_gui/common/toolbars/widget_viewports.hpp"
#include "qt_gui/common/toolbars/widget_render_mode.hpp"

// -----------------------------------------------------------------------------

#include <QToolBar>
#include <QComboBox>

// -----------------------------------------------------------------------------

class Toolbar : public QToolBar {
    Q_OBJECT
public:

    Toolbar(QWidget* parent);

    Widget_selection*   _wgt_select;
    Widget_viewports*   _wgt_viewport;
    Widget_render_mode* _wgt_rd_mode;
    Widget_fitting*     _wgt_fit;
    /// ComboBox to choose the pivot mode of the camera
    QComboBox*          _pivot_comboBox;
    /// Pivot of the gizmo
    QComboBox*          _pivot_gizmo_comboBox;
    /// orientation of the gizmo
    QComboBox*          _dir_gizmo_comboBox;

public slots:

signals:

private:

};

#endif // TOOLBAR_HPP__
