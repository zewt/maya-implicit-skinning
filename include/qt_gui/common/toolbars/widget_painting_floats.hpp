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
#ifndef WIDGET_PAINTING_HPP__
#define WIDGET_PAINTING_HPP__

#include "ui_widget_painting_floats.h"

class Widget_painting : public QWidget, public Ui::Painting_floats {
    Q_OBJECT
public:
    Widget_painting(QWidget* parent) : QWidget(parent)
    {
        setupUi(this);
    }
};


#endif // WIDGET_PAINTING_HPP__
