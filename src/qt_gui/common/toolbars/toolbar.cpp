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
#include "common/toolbars/toolbar.hpp"
#include "common/OGL_widget_enum.hpp"
#include "common/IO_selection_enum.hpp"

Toolbar::Toolbar(QWidget* parent) :
    QToolBar(parent)
{
    // Adding toolButtons for selection
    _wgt_select = new Widget_selection(this);
    this->addWidget( _wgt_select );
    this->addSeparator();

    _wgt_viewport = new Widget_viewports(this);
    this->addWidget( _wgt_viewport );
    this->addSeparator();

    // Adding toolButtons for the rendering mode
    _wgt_rd_mode = new Widget_render_mode(this);
    this->addWidget( _wgt_rd_mode );
    this->addSeparator();

    _wgt_fit = new Widget_fitting( this );
    this->addWidget( _wgt_fit );
    this->addSeparator();

    // Adding combo box for the camera pivot mode
    _pivot_comboBox = new QComboBox( this );
    _pivot_comboBox->setObjectName(QString::fromUtf8("pivot_comboBox"));
    _pivot_comboBox->addItem("Joint"    , (int)EOGL_widget::JOINT     );
    _pivot_comboBox->addItem("Bone"     , (int)EOGL_widget::BONE      );
    _pivot_comboBox->addItem("Selection", (int)EOGL_widget::SELECTION );
    _pivot_comboBox->addItem("User"     , (int)EOGL_widget::USER      );
    _pivot_comboBox->addItem("Free"     , (int)EOGL_widget::FREE      );
    this->addWidget(_pivot_comboBox);

    // Adding combo box for the gizmo pivot mode
    _pivot_gizmo_comboBox = new QComboBox( this );
    _pivot_gizmo_comboBox->setObjectName(QString::fromUtf8("pivot_comboBox") );
    _pivot_gizmo_comboBox->addItem("Median"   , (int)EIO_Selection::MEDIAN   );
    _pivot_gizmo_comboBox->addItem("Active"   , (int)EIO_Selection::ACTIVE   );
    _pivot_gizmo_comboBox->addItem("3D cursor", (int)EIO_Selection::CURSOR_3D);
    this->addWidget(_pivot_gizmo_comboBox);

    // Adding combo box for the gizmo dir mode
    _dir_gizmo_comboBox = new QComboBox( this );
    _dir_gizmo_comboBox->setObjectName(QString::fromUtf8("pivot_comboBox"));
    _dir_gizmo_comboBox->addItem("Local" , (int)EIO_Selection::LOCAL   );
    _dir_gizmo_comboBox->addItem("Global", (int)EIO_Selection::GLOBAL  );
    _dir_gizmo_comboBox->addItem("Normal", (int)EIO_Selection::NORMAL  );
    _dir_gizmo_comboBox->addItem("View"  , (int)EIO_Selection::VIEW    );
    this->addWidget(_dir_gizmo_comboBox);
}
