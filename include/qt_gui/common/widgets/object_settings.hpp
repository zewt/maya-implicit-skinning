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
#ifndef OBJECT_SETTINGS_HPP__
#define OBJECT_SETTINGS_HPP__

#include "ui_object_settings.h"

class Object_settings : public QWidget, public Ui_Object_settings
{
    Q_OBJECT
public:

    Object_settings(QWidget* parent = 0) : QWidget(parent) {
        setupUi( this );
    }

    ~Object_settings(){

    }

};

#endif // OBJECT_SETTINGS_HPP__
