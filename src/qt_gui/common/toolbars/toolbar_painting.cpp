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
#include "common/toolbars/toolbar_painting.hpp"
#include "animesh_enum.hpp"

#include <QLayout>

Toolbar_painting::Toolbar_painting(QWidget *parent) :
    QToolBar(parent)

{
    //Setup layout
    QLayout* layout = this->layout();
    layout->setSpacing(6);
    layout->setObjectName(QString::fromUtf8("_hLayout"));
    //_hLayout->setContentsMargins(-1, 0, -1, 0);
    // -----------------

    _enable_paint = new QCheckBox(this);
    _enable_paint->setText("Enable painting");
    this->addWidget(_enable_paint);

    _paint_mode_comboBox = new QComboBox( this );
    _paint_mode_comboBox->addItem("Implicit skin", (int)EAnimesh::PT_SSD_INTERPOLATION);
    _paint_mode_comboBox->addItem("Cluster"      , (int)EAnimesh::PT_CLUSTER          );
    _paint_mode_comboBox->addItem("Bones weights", (int)EAnimesh::PT_SSD_WEIGHTS      );
    this->addWidget(_paint_mode_comboBox);

    _paint_widget = new Widget_painting(this);
    this->addWidget(_paint_widget);
}

Toolbar_painting::~Toolbar_painting()
{

}
