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
#include "common/widgets/settings_raytracing.hpp"

#include "cuda_ctrl.hpp"

void Settings_raytracing::on_enable_env_map_toggled(bool checked)
{
    Cuda_ctrl::_display._env_map = checked;
    emit update_raytracing();
    emit update_viewports();
}

void Settings_raytracing::on_show_potential_toggled(bool checked)
{
    Cuda_ctrl::_display._potential_2d = checked;
    emit update_raytracing();
    emit update_viewports();
}

void Settings_raytracing::on_enable_bloom_toggled(bool checked)
{
    Cuda_ctrl::_display._bloom = checked;
    emit update_raytracing();
}

void Settings_raytracing::on_enable_lighting_toggled(bool checked)
{
    Cuda_ctrl::_display._raytracing_lighting = checked;
    emit update_raytracing();
    emit update_viewports();
}

void Settings_raytracing::on_progressive_mode_toggled(bool checked)
{
    Cuda_ctrl::_display._progressive_raytracing = checked;
}

void Settings_raytracing::on_enable_raytrace_primitives_toggled(bool checked)
{
    Cuda_ctrl::_display._raytrace_primitives = checked;
    emit update_raytracing();
    emit update_viewports();
}

void Settings_raytracing::on_dSpinB_ray_marching_step_len_valueChanged(double val)
{
    Cuda_ctrl::_display._ray_marching_step_length = val;
}
