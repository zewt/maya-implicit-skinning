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
#ifndef WIDGET_FRAME_HPP__
#define WIDGET_FRAME_HPP__

#include "ui_widget_frame.h"

class Widget_frame : public QWidget, public Ui::Frame_toolbuttons {
    Q_OBJECT
public:
    Widget_frame(QWidget* parent) : QWidget(parent) {
        setupUi(this);
    }

private slots:
    void on_toolB_prev_pressed(){ toolB_prev->setChecked(true); }
    void on_toolB_next_pressed(){ toolB_next->setChecked(true); }
    void on_toolB_play_pressed(){ toolB_play->setChecked(true); }
    void on_toolB_stop_pressed(){ toolB_stop->setChecked(true); }
};

#endif // WIDGET_FRAME_HPP__
