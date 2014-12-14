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
#include "SKIN/main_window_skin.hpp"

#include <QMessageBox>

#include "SKIN/OGL_viewports_skin.hpp"
#include "cuda_ctrl.hpp"

// TODO: to be deleted ////////////////
#include "mesh.hpp"
extern Mesh* g_mesh;
#include "graph.hpp"
extern Graph* g_graph;
#include "skeleton.hpp"
extern Skeleton* g_skel;
///////////////////

////////////////////////////////////////////////////////////
// Implement what's related to display tab of the toolbox //
////////////////////////////////////////////////////////////

void Main_window_skin::on_dSpinB_near_plane_valueChanged(double val)
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    wgl->camera()->set_near(val);
    update_viewports();
}

void Main_window_skin::on_dSpinB_far_plane_valueChanged(double val)
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    wgl->camera()->set_far(val);
    update_viewports();
}

// COLOR MESH ==================================================================

void Main_window_skin::on_ssd_interpolation_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::SSD_INTERPOLATION);
        update_viewports();
    }
}

void Main_window_skin::on_base_potential_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::BASE_POTENTIAL);
        update_viewports();
    }
}

void Main_window_skin::on_cluster_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::CLUSTER);
        update_viewports();
    }
}

void Main_window_skin::on_color_grey_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_uniform(0.8f, 0.8f, 0.8f, 0.99f);
        update_viewports();
    }
}

void Main_window_skin::on_color_smoothing_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::SMOOTHING_WEIGHTS);
        update_viewports();
    }
}

void Main_window_skin::on_ssd_weights_toggled(bool checked)
{
    if(checked)
    {
        const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

        if(set.size() > 0)
        {
            int id = set[set.size()-1];
            Cuda_ctrl::_anim_mesh->color_ssd_weights(id);
            update_viewports();
        }
    }
}

void Main_window_skin::on_color_nearest_joint_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::NEAREST_JOINT);
        update_viewports();
    }
}

void Main_window_skin::on_implicit_gradient_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::GRAD_POTENTIAL);
        update_viewports();
    }
}

void Main_window_skin::on_color_normals_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::NORMAL);
        update_viewports();
    }
}

void Main_window_skin::on_vertices_state_toggled(bool checked)
{
    if( checked )
    {
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::VERTICES_STATE);
        update_viewports();
    }
}

void Main_window_skin::on_buton_uniform_point_cl_toggled(bool checked)
{
    if( checked )
    {
        Color cl = Cuda_ctrl::_color.get(Color_ctrl::MESH_POINTS);
        g_mesh->set_point_color_bo(cl.r, cl.g, cl.b, cl.a);
        update_viewports();
    }
}

// END COLOR MESH ==============================================================

void Main_window_skin::on_display_skeleton_toggled(bool checked)
{
    Cuda_ctrl::_skeleton.switch_display();
    update_viewports();
}

void Main_window_skin::on_display_operator_toggled(bool checked)
{
    Cuda_ctrl::_operators.set_display_operator(checked);
    update_viewports();
}

void Main_window_skin::on_display_controller_toggled(bool checked)
{
    Cuda_ctrl::_operators.set_display_controller(checked);
    update_viewports();
}

// RAYTRACING ==================================================================

void Main_window_skin::settings_raytracing_on_enable_raytracing_toggled(bool checked)
{
    _viewports->active_viewport()->set_raytracing( checked );
    update_viewports();
}

void Main_window_skin::settings_raytracing_on_potential_plane_pos_released()
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    Camera* cam = wgl->camera();

    using namespace Cuda_ctrl;
    Vec3_cu up  = cam->get_y().normalized();
    Vec3_cu dir = cam->get_dir().normalized();
    Vec3_cu pos = cam->get_pos();

#if 1
    _potential_plane._normal.set(up.x, up.y, up.z);
    Vec3_cu v = (pos + dir * cam->get_near()) + up * 0.01f;
    _potential_plane._org.set(v.x, v.y, v.z);
#else
    //DEBUG/////////////
    _potential_plane._normal.set(dir.x, dir.y, dir.z);
    _potential_plane._org.set(0.f, 0.f, 0.f);
    //DEBUG/////////////
#endif

    _potential_plane._setup = true;
    wgl->updateGL();
}

// END RAYTRACING ==============================================================

void Main_window_skin::on_wireframe_toggled(bool checked)
{
    Cuda_ctrl::_display._wire = checked;
    update_viewports();
}

void Main_window_skin::on_display_oriented_bbox_toggled(bool checked)
{
    Cuda_ctrl::_display._oriented_bbox = checked;
    update_viewports();
}

void Main_window_skin::on_spinBox_valueChanged(int val)
{
    Cuda_ctrl::_operators.set_display_size(val);
    update_viewports();
}

void Main_window_skin::on_comboB_operators_currentIndexChanged(int idx)
{
    int t = comboB_operators->itemData(idx).toInt();
    Cuda_ctrl::_display._operator_type = Blending_env::Op_t(t);
//    Cuda_ctrl::_display._operator_mode = Blending_env::Op_mode(m); // TODO

    Cuda_ctrl::_operators.update_displayed_operator_texture();

    update_viewports();
}

void Main_window_skin::on_dSpinB_opening_value_valueChanged(double val)
{
    Cuda_ctrl::_display._opening_angle = val;
    Cuda_ctrl::_operators.update_displayed_operator_texture();
    update_viewports();
}

void Main_window_skin::on_spinB_aperture_valueChanged(int val)
{
    Vec_viewports list = _viewports->get_viewports();
    for(unsigned i = 0; i < list.size(); i++)
        list[i]->camera()->set_fov_deg((float)val);

    update_viewports();
}

void Main_window_skin::on_pushB_reset_camera_released()
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    Vec3_cu p = Vec3_cu::zero();
    wgl->camera()->set_pos(p.x, p.y, p.z);
    wgl->camera()->set_dir( Vec3_cu::unit_z() );
    update_viewports();
}

void Main_window_skin::on_checkB_camera_tracking_toggled(bool checked)
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    wgl->_track_pivot = checked;
    update_viewports();
}
