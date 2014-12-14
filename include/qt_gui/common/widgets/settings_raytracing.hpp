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
#ifndef SETTINGS_RAYTRACING_HPP__
#define SETTINGS_RAYTRACING_HPP__

#include "ui_settings_raytracing.h"

class Settings_raytracing : public QWidget, public Ui_Settings_raytracing
{
    Q_OBJECT
public:

    Settings_raytracing(QWidget* parent = 0) : QWidget(parent) {
        setupUi( this );
    }

    ~Settings_raytracing(){    }

signals:
    emit void update_viewports();
    emit void update_raytracing();

private slots:
//    void on_enable_raytracing_toggled(bool checked);

    void on_enable_env_map_toggled(bool checked);
    void on_show_potential_toggled(bool checked);
    void on_enable_bloom_toggled(bool checked);
    void on_enable_lighting_toggled(bool checked);
    void on_progressive_mode_toggled(bool checked);
//    void on_potential_plane_pos_released();
    void on_enable_raytrace_primitives_toggled(bool checked);
    void on_dSpinB_ray_marching_step_len_valueChanged(double val);

};

#endif // SETTINGS_RAYTRACING_HPP__
