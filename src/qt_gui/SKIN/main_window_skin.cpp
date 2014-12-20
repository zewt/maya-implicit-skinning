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

#include <QFileDialog>
#include <QColorDialog>
#include <QMessageBox>
#include <QString>

#include "blending_lib/generator.hpp"
#include "common/tools/popup_ok_cancel.hpp"
#include "vec3_cu.hpp"
#include "camera.hpp"
#include "display_operator.hpp"
#include "SKIN/OGL_viewports_skin.hpp"
#include "cuda_ctrl.hpp"

// TODO: to be deleted ////////////////
extern Mesh* g_mesh;
#include "graph.hpp"
extern Graph* g_graph;
#include "skeleton.hpp"
extern Skeleton* g_skel;
///////////////////


extern std::string g_icons_dir;

using namespace Cuda_ctrl;

// -----------------------------------------------------------------------------

Main_window_skin::Main_window_skin(QWidget *parent) :
    QMainWindow(parent)
{
    //--------------
    setupUi(this);
    //--------------
    setup_viewports();
    setup_comboB_operators();
    setup_main_window();
    //--------------

    // HACK: Because blending_env initialize to elbow too ...
    IBL::Ctrl_setup shape = IBL::Shape::elbow();
    update_ctrl_spin_boxes(shape);

    // Desactivate GUI parts
    enable_animesh( false );
    enable_mesh   ( false );
}

// -----------------------------------------------------------------------------

void Main_window_skin::setup_comboB_operators()
{
    comboB_operators->addItem("Max union 2D"           , (int)Blending_env::MAX     );
    comboB_operators->addItem("Bulge 4D"               , (int)Blending_env::B_OH_4D );
    comboB_operators->addItem("Bulge 3D"               , (int)Blending_env::B_D     );
    comboB_operators->addItem("Ultimate union 3D"      , (int)Blending_env::U_OH    );
    comboB_operators->addItem("Diamond union 3D"       , (int)Blending_env::C_D     );
}

// -----------------------------------------------------------------------------

void Main_window_skin::setup_viewports()
{
    _viewports = new OGL_viewports_skin(viewports_frame, this);
    viewports_frame->layout()->addWidget(_viewports);
}

// -----------------------------------------------------------------------------

void Main_window_skin::setup_main_window()
{
    setWindowTitle("Implicit skinning ");
    resize(1300, 800);
    QIcon icon(QString(g_icons_dir.c_str())+"/logo/red_logo_white.svg");
    QWidget::setWindowIcon( icon );
}

// -----------------------------------------------------------------------------

void Main_window_skin::choose_hrbf_samples(int bone_id)
{
    if( g_skel->is_leaf(bone_id) ) return;

    switch( cBox_sampling_type->currentIndex())
    {
    case 0:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_poisson
                (bone_id,
                 dSpinB_max_dist_joint->value(),
                 dSpinB_max_dist_parent->value(),
                 dSpinB_min_dist_samples->value(),
                 spinB_nb_samples_psd->value(),
                 dSpinB_max_fold->value() );

    }break;

    case 1:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_ad_hoc
                (bone_id,
                 dSpinB_max_dist_joint->value(),
                 dSpinB_max_dist_parent->value(),
                 dSpinB_min_dist_samples->value(),
                 dSpinB_max_fold->value() );

    }break;

    case 2:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_gael( bone_id );
        break;
    }
    }
}

// -----------------------------------------------------------------------------

void Main_window_skin::choose_hrbf_samples_selected_bones()
{

    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    for(unsigned i = 0; i < set.size(); i++)
        choose_hrbf_samples(set[i]);

    Cuda_ctrl::_anim_mesh->update_base_potential();
}

// -----------------------------------------------------------------------------

void Main_window_skin::update_ctrl_spin_boxes(const IBL::Ctrl_setup& shape)
{
    {
        dSpinB_ctrl_p0_x->blockSignals(true);
        dSpinB_ctrl_p0_y->blockSignals(true);
        dSpinB_ctrl_p1_x->blockSignals(true);
        dSpinB_ctrl_p1_y->blockSignals(true);
        dSpinB_ctrl_p2_x->blockSignals(true);
        dSpinB_ctrl_p2_y->blockSignals(true);
        dSpinB_ctrl_slope0->blockSignals(true);
        dSpinB_ctrl_slope1->blockSignals(true);
    }

    dSpinB_ctrl_p0_x->setValue(shape.p0().x);
    dSpinB_ctrl_p0_y->setValue(shape.p0().y);

    dSpinB_ctrl_p1_x->setValue(shape.p1().x);
    dSpinB_ctrl_p1_y->setValue(shape.p1().y);

    dSpinB_ctrl_p2_x->setValue(shape.p2().x);
    dSpinB_ctrl_p2_y->setValue(shape.p2().y);

    dSpinB_ctrl_slope0->setValue(shape.s0());
    dSpinB_ctrl_slope1->setValue(shape.s1());

    {
        dSpinB_ctrl_p0_x->blockSignals(false);
        dSpinB_ctrl_p0_y->blockSignals(false);
        dSpinB_ctrl_p1_x->blockSignals(false);
        dSpinB_ctrl_p1_y->blockSignals(false);
        dSpinB_ctrl_p2_x->blockSignals(false);
        dSpinB_ctrl_p2_y->blockSignals(false);
        dSpinB_ctrl_slope0->blockSignals(false);
        dSpinB_ctrl_slope1->blockSignals(false);
    }
}

// -----------------------------------------------------------------------------

void Main_window_skin::set_current_ctrl()
{
    IBL::float2 p0;
    p0.x = dSpinB_ctrl_p0_x->value();
    p0.y = dSpinB_ctrl_p0_y->value();

    IBL::float2 p1;
    p1.x = dSpinB_ctrl_p1_x->value();
    p1.y = dSpinB_ctrl_p1_y->value();

    IBL::float2 p2;
    p2.x = dSpinB_ctrl_p2_x->value();
    p2.y = dSpinB_ctrl_p2_y->value();

    IBL::float2 s;
    s.x = dSpinB_ctrl_slope0->value();
    s.y = dSpinB_ctrl_slope1->value();

    IBL::Ctrl_setup shape(p0, p1, p2, s.x, s.y);

    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    if(set.size() > 0)
        for(unsigned i = 0; i < set.size(); i++)
            _skeleton.set_joint_controller(set[i], shape);
    else
        _operators.set_global_controller(shape);

    _anim_mesh->update_base_potential();
}

// -----------------------------------------------------------------------------

void Main_window_skin::paint_toggled(bool state)
{
}

// MANUAL SLOTS ################################################################

void Main_window_skin::show_all_gizmo(bool checked) {}
void Main_window_skin::set_gizmo_trans() {}
void Main_window_skin::set_gizmo_rot() {}
void Main_window_skin::set_gizmo_trackball() {}
void Main_window_skin::set_gizmo_scale() {}

void Main_window_skin::toggle_fitting(bool checked){
    Cuda_ctrl::_anim_mesh->set_implicit_skinning(checked);
}

void Main_window_skin::active_viewport(int id)
{
}

// AUTOMATIC SLOTS #############################################################

void Main_window_skin::on_actionExit_triggered()
{
    exit(0);
}

// SMOOTHING SLOTS =============================================================

void Main_window_skin::on_enable_smoothing_toggled(bool checked){
    Cuda_ctrl::_anim_mesh->set_do_smoothing(checked);
}

void Main_window_skin::on_spinBox_smooth_iter_valueChanged(int val){
    Cuda_ctrl::_anim_mesh->set_nb_iter_smooting(val);
}

void Main_window_skin::on_spinB_smooth_force_a_valueChanged(double val){
    Cuda_ctrl::_anim_mesh->set_smooth_force_a(val);
}

void Main_window_skin::on_checkB_enable_smoothing_toggled(bool checked)
{
    Cuda_ctrl::_debug._smooth_mesh = checked;
}

void Main_window_skin::on_spinB_nb_iter_smooth1_valueChanged(int val)
{
    Cuda_ctrl::_debug._smooth1_iter = val;
}

void Main_window_skin::on_dSpinB_lambda_smooth1_valueChanged(double val)
{
    Cuda_ctrl::_debug._smooth1_force = val;
}

void Main_window_skin::on_spinB_nb_iter_smooth2_valueChanged(int val)
{
    Cuda_ctrl::_debug._smooth2_iter = val;
}

void Main_window_skin::on_dSpinB_lambda_smooth2_valueChanged(double val)
{
    Cuda_ctrl::_debug._smooth2_force= val;
}

// END SMOOTHING SLOTS =========================================================

void Main_window_skin::on_horizontalSlider_sliderMoved(int position)
{
    Cuda_ctrl::_display.set_transparency_factor( (float)position/100.f );
}

void Main_window_skin::on_ssd_raio_toggled(bool checked)
{
    if(checked)
    {
        Cuda_ctrl::_anim_mesh->do_ssd_skinning();
    }
}

void Main_window_skin::on_dual_quaternion_radio_toggled(bool checked)
{
    if(checked)
    {
        Cuda_ctrl::_anim_mesh->do_dual_quat_skinning();
    }
}

void Main_window_skin::on_implicit_skinning_checkBox_toggled(bool checked)
{
    Cuda_ctrl::_anim_mesh->set_implicit_skinning(checked);
}

// BLENDING SLOTS ==============================================================

void Main_window_skin::on_spinBox_bulge_in_contact_force_valueChanged(double mag)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();
    if(set.size() > 0){
        Cuda_ctrl::_skeleton.set_joint_bulge_mag(set[set.size()-1], mag);
    }
    else
        Cuda_ctrl::_operators.set_bulge_magnitude((float)mag);

    Cuda_ctrl::_operators.update_displayed_operator_texture();
    Cuda_ctrl::_display._raytrace_again = true;
}

void Main_window_skin::on_update_bulge_in_contact_released()
{
    Cuda_ctrl::_operators.update_bulge();
}

// END BLENDING SLOTS ==========================================================

// RBF EDITION SLOTS ===========================================================

void Main_window_skin::on_rbf_edition_toggled(bool checked)
{
    Cuda_ctrl::_display._edit_hrbf_samples = checked;
}

void Main_window_skin::on_local_frame_toggled(bool){
}

void Main_window_skin::on_checkB_align_with_normal_toggled(bool){
}

void Main_window_skin::on_checkB_factor_siblings_toggled(bool checked)
{
    Cuda_ctrl::_anim_mesh->set_factor_siblings( checked );
}

// END RBF EDITION SLOTS =======================================================

// HELP MENU ===================================================================

void Main_window_skin::on_actionShortcuts_triggered()
{
}

void Main_window_skin::on_actionAbout_triggered()
{
/*                             "Authors",
                             "Coding, conception && research :\n"
                             "(alphabetical order of first names)\n"
                             "Damien Rohmer, Florian Canezin, Gael Guennebaud, Loic Barthe, Olivier Gourmel, Rodolphe Vaillant\n"
                             "Logo, theme && icons\n"
                             "Judith Belin\n");
                             */
}

// END HELP MENU ===============================================================

// INFOS =======================================================================

void Main_window_skin::on_actionSkeleton_triggered()
{
    if(g_mesh != 0)
    {
        Skeleton_ctrl& skel = Cuda_ctrl::_skeleton;
        QMessageBox::information(this,
                                 "Skeleton informations",
                                 "Nb joint: "+QString::number(skel.get_nb_joints())+"\n"+
                                 "Hierachy: \n"+
                                 QString( g_skel->to_string().c_str() )
                                 );
    }
    else
    {
        QMessageBox::information(this, "Skeleton informations", "No skeleton to get infos from");
    }
}

void Main_window_skin::on_actionMesh_triggered()
{
    if(g_mesh != 0)
    {
        QMessageBox::information(this,
                                 "Mesh informations",
                                 "Nb vertices: "+QString::number(g_mesh->get_nb_vertices())+"\n"
                                 "Nb faces: "+QString::number(g_mesh->get_nb_faces())+"\n"
                                 "Nb triangles: "+QString::number(g_mesh->get_nb_tri())+"\n"
                                 "Nb quads: "+QString::number(g_mesh->get_nb_quad())+"\n"
                                 "Manifold ? "+ QString(g_mesh->is_manifold() ? "Yes" : "No") +"\n"
                                 "Closed ? "+ QString(g_mesh->is_closed() ? "Yes" : "No") +"\n"
                                 "Defined materials ? "+ QString(g_mesh->has_materials() ? "Yes" : "No") +"\n"
                                 "Bump map ? "+ QString(g_mesh->has_bumpmap() ? "Yes" : "No") +"\n"
                                 );
    }
    else
    {
        QMessageBox::information(this, "Mesh informations", "No mesh to get infos from");
    }
}

void Main_window_skin::on_actionResource_usage_triggered()
{
    double free, total;
    Cuda_ctrl::get_mem_usage(total, free);
    QMessageBox::information(this,
                             "Resource usage",
                             "Total device mem:   "+QString::number(total)+"MB\n"
                             "Used device mem:   "+QString::number(total-free)+"MB\n"
                             "Free device mem:   "+QString::number(free)+"MB\n"
                             );
}

// END INFOS ===================================================================

void Main_window_skin::on_reset_anim_released()
{
    Cuda_ctrl::_skeleton.reset();
}

void Main_window_skin::on_enable_partial_fit_toggled(bool checked)
{
    Cuda_ctrl::_debug._do_partial_fit = checked;
}

void Main_window_skin::on_spinBox_nb_step_fitting_valueChanged(int val)
{
    Cuda_ctrl::_debug._nb_step = val;
}

void Main_window_skin::on_debug_show_normal_toggled(bool checked)
{
    Cuda_ctrl::_debug._show_normals = checked;
}

void Main_window_skin::on_debug_show_gradient_toggled(bool checked)
{
    Cuda_ctrl::_debug._show_gradient = checked;
}

void Main_window_skin::on_actionColors_triggered()
{
}

void Main_window_skin::on_doubleSpinBox_valueChanged(double val)
{
    Cuda_ctrl::_debug._collision_threshold = val;
}

void Main_window_skin::on_box_potential_pit_toggled(bool checked)
{
    Cuda_ctrl::_debug._potential_pit = checked;
}

void Main_window_skin::on_spinBox_diffuse_smoothing_weights_iter_valueChanged(int val)
{
    Cuda_ctrl::_anim_mesh->set_smoothing_weights_diffusion_iter(val);
}

void Main_window_skin::on_actionReloadShaders_triggered()
{
    Cuda_ctrl::reload_shaders();
}

void Main_window_skin::on_button_defects_point_cl_toggled(bool checked)
{
    if( checked )
    {
        Color cl = Cuda_ctrl::_color.get(Color_ctrl::MESH_POINTS);
        g_mesh->set_point_color_bo(cl.r, cl.g, cl.b, cl.a);

        cl = Cuda_ctrl::_color.get(Color_ctrl::MESH_DEFECTS);
        const std::vector<int>& list0 = g_mesh->get_not_manifold_list();
        const std::vector<int>& list1 = g_mesh->get_on_side_list();

        float* color_ptr = 0;
        g_mesh->_point_color_bo.map_to(color_ptr, GL_WRITE_ONLY);
        for(unsigned i = 0; i < (list0.size()+list1.size()); i++)
        {
            int v_idx = i < list0.size() ? list0[i] : list1[i-list0.size()];
            const Mesh::Packed_data d = g_mesh->get_packed_vert_map()[v_idx];
            for(int j = 0; j < d.nb_ocurrence; j++)
            {
                const int p_idx = d.idx_data_unpacked + j;
                color_ptr[p_idx*4  ] = cl.r;
                color_ptr[p_idx*4+1] = cl.g;
                color_ptr[p_idx*4+2] = cl.b;
                color_ptr[p_idx*4+3] = cl.a;
            }
        }
        g_mesh->_point_color_bo.unmap();
    }
}

void Main_window_skin::on_spinB_step_length_valueChanged(double val)
{
    Cuda_ctrl::_debug._step_length = val;
}

void Main_window_skin::on_checkBox_collsion_on_toggled(bool checked)
{
    Cuda_ctrl::_debug._fit_on_all_bones = checked;
}

void Main_window_skin::on_pushB_attached_skeleton_released()
{
    if( g_graph != 0 && g_graph->_vertices.size() > 0 &&
        Cuda_ctrl::is_mesh_loaded() )
    {

        Cuda_ctrl::_skeleton.load( *g_graph );
        Cuda_ctrl::load_animesh();
        enable_animesh( true );
    }
}

void Main_window_skin::on_pushB_set_rigid_weights_released()
{
    Cuda_ctrl::_anim_mesh->init_rigid_ssd_weights();
}

void Main_window_skin::on_pushB_diffuse_curr_weights_released()
{
    Cuda_ctrl::_anim_mesh->topology_diffuse_ssd_weights(dSpinB_diff_w_alpha->value(), spinB_diff_w_nb_iter->value());
}

void Main_window_skin::on_pushB_diff_w_exp_released()
{
    Cuda_ctrl::_anim_mesh->geodesic_diffuse_ssd_weights(dSpinB_diff_w_alpha_exp->value(), spinB_auto_w_nb_iter_exp->value());
}

void Main_window_skin::on_choose_hrbf_samples_released()
{
    choose_hrbf_samples_selected_bones();
}

void Main_window_skin::on_checkB_show_junction_toggled(bool checked)
{
    Cuda_ctrl::_display._junction_spheres = checked;
}

void Main_window_skin::on_dSpinB_min_dist_samples_valueChanged(double )
{
    if(checkB_auto_sample->isChecked())
    {
        choose_hrbf_samples_selected_bones();
    }
}

void Main_window_skin::on_dSpinB_max_fold_valueChanged(double )
{
    if(checkB_auto_sample->isChecked())
    {
        choose_hrbf_samples_selected_bones();
    }
}

void Main_window_skin::on_dSpinB_max_dist_joint_valueChanged(double )
{
    if(checkB_auto_sample->isChecked())
    {
        choose_hrbf_samples_selected_bones();
    }
}

void Main_window_skin::on_dSpinB_max_dist_parent_valueChanged(double )
{
    if(checkB_auto_sample->isChecked())
    {
        choose_hrbf_samples_selected_bones();
    }
}

void Main_window_skin::on_dSpinB_collision_depth_valueChanged(double val)
{
    Cuda_ctrl::_debug._collision_depth = val;
}

// CONTROLLER SPINBOXES ========================================================

void Main_window_skin::on_dSpinB_ctrl_p0_x_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
}

void Main_window_skin::on_dSpinB_ctrl_p0_y_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
}

void Main_window_skin::on_dSpinB_ctrl_p1_x_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
}

void Main_window_skin::on_dSpinB_ctrl_p1_y_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
}

void Main_window_skin::on_dSpinB_ctrl_p2_x_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
}

void Main_window_skin::on_dSpinB_ctrl_p2_y_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
}

void Main_window_skin::on_dSpinB_ctrl_slope0_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
}

void Main_window_skin::on_dSpinB_ctrl_slope1_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
}

// END CONTROLLER SPINBOXES ====================================================

void Main_window_skin::on_checkB_cap_joint_toggled(bool checked)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->set_jcap(set[i], checked);
}

void Main_window_skin::on_checkB_capparent_toggled(bool checked)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->set_pcap(set[i], checked);
}

void Main_window_skin::on_dSpinB_hrbf_radius_valueChanged(double val)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();
    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->set_hrbf_radius(set[i], val);

    Cuda_ctrl::_anim_mesh->update_base_potential();
}

void Main_window_skin::on_checkBox_update_base_potential_toggled(bool checked)
{
    Cuda_ctrl::_anim_mesh->enable_update_base_potential( checked );
}

void Main_window_skin::on_pButton_compute_heat_difusion_released()
{
//    if(g_mesh->is_manifold() && g_mesh->is_closed() && g_mesh->get_nb_quad() == 0)
    {
        _anim_mesh->heat_diffuse_ssd_weights( dSpinBox_heat_coeff->value() );
    }
//    else
//    {
//        QMessageBox::information(this,
//                                 "Error",
//                                 "To compute heat diffusion mesh should be\n"
//                                 "closed 2-manifold and triangular\n");
//    }
}

void Main_window_skin::on_cBox_always_precompute_toggled(bool checked)
{
    Cuda_ctrl::_anim_mesh->set_auto_precompute( checked );
}

void Main_window_skin::on_spinB_max_res_valueChanged(int val)
{
    Cuda_ctrl::_display._nb_samples_res = val;
}


void Main_window_skin::on_pushB_empty_bone_released()
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();
    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->empty_samples( set[i] );
}

static void set_caps_selected_joints(bool state)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    for(unsigned i = 0; i < set.size(); i++)
    {
        const int idx = set[i];
        if( g_skel->is_leaf(idx) ) continue;

        Cuda_ctrl::_anim_mesh->set_pcap(idx, state);

        int pt = Cuda_ctrl::_skeleton.get_parent( idx );
        if( pt > -1) Cuda_ctrl::_anim_mesh->set_jcap(pt, state);

        //const std::vector<int>& sons = Cuda_ctrl::_skeleton.get_sons(idx);
        //for(unsigned s = 0; s < sons.size(); s++)
    }
    Cuda_ctrl::_anim_mesh->update_base_potential();
}

void Main_window_skin::on_pButton_add_caps_released()
{
    set_caps_selected_joints(true);
}

void Main_window_skin::on_pButton_supr_caps_released()
{
    set_caps_selected_joints(false);
}

void Main_window_skin::on_spinBox_2_valueChanged(int val)
{
    Cuda_ctrl::_debug._slope_smooth_weight = val % 2 == 0 ? val : val + 1;
}

void Main_window_skin::on_checkB_enable_raphson_toggled(bool checked)
{
    Cuda_ctrl::_debug._raphson = checked;
}

void Main_window_skin::on_color_smoothing_conservative_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::ANIM_SMOOTH_CONSERVATIVE);
    }
}

void Main_window_skin::on_color_smoothing_laplacian_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::ANIM_SMOOTH_LAPLACIAN);
    }
}

// Forward def
namespace Skeleton_env {
void set_grid_res(Skel_id i, int res);
}
// END Forward def

void Main_window_skin::on_spinB_grid_res_valueChanged(int res)
{
    Skeleton_env::set_grid_res(0, res); // FIXME: remove hardcoded skeleton ID
}

void Main_window_skin::on_checkB_aa_bbox_clicked(bool checked)
{
    Cuda_ctrl::_display._aa_bbox= checked;
}


void Main_window_skin::enable_animesh(bool state)
{
}

// -----------------------------------------------------------------------------

void Main_window_skin::enable_mesh(bool state)
{
    box_mesh_color->setEnabled( state );
}

void Main_window_skin::keyPressEvent( QKeyEvent* event )
{

}












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
}

void Main_window_skin::on_dSpinB_far_plane_valueChanged(double val)
{
}

// COLOR MESH ==================================================================

void Main_window_skin::on_ssd_interpolation_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::SSD_INTERPOLATION);
    }
}

void Main_window_skin::on_base_potential_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::BASE_POTENTIAL);
    }
}

void Main_window_skin::on_cluster_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::CLUSTER);
    }
}

void Main_window_skin::on_color_grey_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_uniform(0.8f, 0.8f, 0.8f, 0.99f);
    }
}

void Main_window_skin::on_color_smoothing_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::SMOOTHING_WEIGHTS);
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
        }
    }
}

void Main_window_skin::on_color_nearest_joint_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::NEAREST_JOINT);
    }
}

void Main_window_skin::on_implicit_gradient_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::GRAD_POTENTIAL);
    }
}

void Main_window_skin::on_color_normals_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::NORMAL);
    }
}

void Main_window_skin::on_vertices_state_toggled(bool checked)
{
    if( checked )
    {
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::VERTICES_STATE);
    }
}

void Main_window_skin::on_buton_uniform_point_cl_toggled(bool checked)
{
}

// END COLOR MESH ==============================================================

void Main_window_skin::on_display_skeleton_toggled(bool checked)
{
}

void Main_window_skin::on_display_operator_toggled(bool checked)
{
}

void Main_window_skin::on_display_controller_toggled(bool checked)
{
}

// RAYTRACING ==================================================================

void Main_window_skin::settings_raytracing_on_enable_raytracing_toggled(bool checked)
{
}

void Main_window_skin::settings_raytracing_on_potential_plane_pos_released()
{
}

// END RAYTRACING ==============================================================

void Main_window_skin::on_wireframe_toggled(bool checked)
{
}

void Main_window_skin::on_display_oriented_bbox_toggled(bool checked)
{
}

void Main_window_skin::on_spinBox_valueChanged(int val)
{
}

void Main_window_skin::on_comboB_operators_currentIndexChanged(int idx)
{
    int t = comboB_operators->itemData(idx).toInt();
    Cuda_ctrl::_display._operator_type = Blending_env::Op_t(t);
//    Cuda_ctrl::_display._operator_mode = Blending_env::Op_mode(m); // TODO

    Cuda_ctrl::_operators.update_displayed_operator_texture();
}

void Main_window_skin::on_dSpinB_opening_value_valueChanged(double val)
{
    Cuda_ctrl::_display._opening_angle = val;
    Cuda_ctrl::_operators.update_displayed_operator_texture();
}

void Main_window_skin::on_spinB_aperture_valueChanged(int val)
{
}

void Main_window_skin::on_pushB_reset_camera_released()
{
}

void Main_window_skin::on_checkB_camera_tracking_toggled(bool checked)
{
}
