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
#ifndef TOOLBAR_PAINTING_HPP__
#define TOOLBAR_PAINTING_HPP__

#include <QToolBar>
#include <QComboBox>
#include <QCheckBox>

#include "common/toolbars/widget_painting_floats.hpp"


class Toolbar_painting : public QToolBar {
    Q_OBJECT

public:
    Toolbar_painting(QWidget *parent);
    ~Toolbar_painting();

    bool is_paint_on() const { return _enable_paint->isChecked(); }

    QCheckBox* _enable_paint;
    Widget_painting* _paint_widget;

public slots:

private:
    QComboBox* _paint_mode_comboBox;
};




#endif // TOOLBAR_PAINTING_HPP__
