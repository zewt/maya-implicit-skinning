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

    Fbx_loader::init();

    // Desactivate GUI parts
    enable_mesh   ( false );
}

// -----------------------------------------------------------------------------

void Main_window_skin::setup_comboB_operators()
{
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
                 // Set a distance threshold from sample to the joints to choose them.
                 -0.02f, // dSpinB_max_dist_joint->value(),
                 -0.02f, // dSpinB_max_dist_parent->value(),
                 0, // dSpinB_min_dist_samples->value(),
                 // Minimal number of samples.  (this value is used only whe the value min dist is zero)
                 50, // spinB_nb_samples_psd->value(), 20-1000

                 // We choose a sample if: max fold > (vertex orthogonal dir to the bone) dot (vertex normal)
                 0); // dSpinB_max_fold->value() );

    }break;

    case 1:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_ad_hoc
                (bone_id,
                 -0.02f, // dSpinB_max_dist_joint->value(),
                 -0.02f, // dSpinB_max_dist_parent->value(),
                 0, // dSpinB_min_dist_samples->value(), Minimal distance between two HRBF sample
                 0); // dSpinB_max_fold->value() );

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

void Main_window_skin::toggle_fitting(bool checked){
    Cuda_ctrl::_anim_mesh->set_implicit_skinning(checked);
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

void Main_window_skin::on_checkB_factor_siblings_toggled(bool checked)
{
    Cuda_ctrl::_anim_mesh->set_factor_siblings( checked );
}

// END RBF EDITION SLOTS =======================================================

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
}

void Main_window_skin::on_debug_show_gradient_toggled(bool checked)
{
}

void Main_window_skin::on_doubleSpinBox_valueChanged(double val)
{
    // When the scalar product between the gradient of step n and n-1 exceed this threshold we stop the vertex.
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
    if( g_graph != 0 && g_graph->_vertices.size() > 0 && Cuda_ctrl::is_mesh_loaded() )
    {
        Cuda_ctrl::_skeleton.load( *g_graph );
        Cuda_ctrl::load_animesh();
    }
}

void Main_window_skin::on_choose_hrbf_samples_released()
{
    choose_hrbf_samples_selected_bones();
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

    // Add hrbf sample to the tip of the selected joint
    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->set_jcap(set[i], checked);
}

void Main_window_skin::on_checkB_capparent_toggled(bool checked)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    // Add hrbf sample to the tip of the parent of the selected joint.
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


// -----------------------------------------------------------------------------

void Main_window_skin::enable_mesh(bool state)
{
}

void Main_window_skin::keyPressEvent( QKeyEvent* event )
{

}












void Main_window_skin::on_spinBox_valueChanged(int val)
{
}

void Main_window_skin::on_comboB_operators_currentIndexChanged(int idx)
{

//    comboB_operators->addItem("Max union 2D"           , (int)Blending_env::MAX     );
//    comboB_operators->addItem("Bulge 4D"               , (int)Blending_env::B_OH_4D );
//    comboB_operators->addItem("Bulge 3D"               , (int)Blending_env::B_D     );
//    comboB_operators->addItem("Ultimate union 3D"      , (int)Blending_env::U_OH    );
//    comboB_operators->addItem("Diamond union 3D"       , (int)Blending_env::C_D     );



    Cuda_ctrl::_display._operator_type = Blending_env::MAX;
//    Cuda_ctrl::_display._operator_mode = Blending_env::Op_mode(m); // TODO

    Cuda_ctrl::_operators.update_displayed_operator_texture();
}

void Main_window_skin::on_dSpinB_opening_value_valueChanged(double val)
{
    Cuda_ctrl::_display._opening_angle = val;
    Cuda_ctrl::_operators.update_displayed_operator_texture();
}











#include <QFileDialog>
#include <QMessageBox>

#include <cassert>
#include <algorithm>

#include "cuda_ctrl.hpp"
#include "vec3_cu.hpp"
#include "gl_mesh.hpp"
#include "SKIN/OGL_viewports_skin.hpp"
#include "loader.hpp"
#include "fbx_loader.hpp"
#include "obj_loader.hpp"
#include "conversions.hpp"

void Main_window_skin::load_fbx_mesh( Fbx_loader::Fbx_file& loader)
{
    Mesh* ptr_mesh = new Mesh();
    Loader::Abs_mesh mesh;
    loader.get_mesh( mesh );
    if(mesh._vertices.size() == 0){
        QMessageBox::information(this, "Warning", "no mesh found");
        return;
    }

    ptr_mesh->load( mesh, loader._path);
    Cuda_ctrl::load_mesh( ptr_mesh );
}

// -----------------------------------------------------------------------------

bool Main_window_skin::load_custom_skeleton(QString name)
{
    QString skel_name = name;
    skel_name.append(".skel");
    if( !QFile::exists(skel_name) )
    {
        QMessageBox::information(this, "Error", "Can't' find "+name+".skel");
        return false;
    }

    Cuda_ctrl::_graph.load_from_file(skel_name.toLatin1());
    Cuda_ctrl::_skeleton.load( *g_graph );
    return true;
}

// -----------------------------------------------------------------------------

bool Main_window_skin::load_custom_weights(QString name)
{
    QString ssd_name = name;
    ssd_name.append(".weights");
    if( QFile::exists(ssd_name) )
    {
        Cuda_ctrl::load_animesh_and_ssd_weights(ssd_name.toLatin1());
        return true;
    }
    else
    {
        QMessageBox::information(this, "Error", "Can't' find "+name+".weights");
        return false;
    }
}

// -----------------------------------------------------------------------------

bool Main_window_skin::load_fbx_skeleton_anims(const Fbx_loader::Fbx_file& loader)
{
    // Extract skeleton data
    Loader::Abs_skeleton skel;
    loader.get_skeleton(skel);
    if(skel._bones.size() == 0) return false;

    // Convert to our skeleton representation
    Cuda_ctrl::_skeleton.load( skel );
    Cuda_ctrl::_skeleton.set_offset_scale( g_mesh->get_offset(), g_mesh->get_scale());

    Cuda_ctrl::load_animesh(); // Bind animated mesh to skel
    // Convert bones weights to our representation
    Cuda_ctrl::_anim_mesh->set_ssd_weight( skel );

    // Load first animation
//    std::vector<Loader::Base_anim_eval*> anims;
//    loader.get_animations( anims );
//    toolBar_frame->set_anim_list( anims );

    return true;
}

void Main_window_skin::on_actionLoad_skeleton_triggered()
{
    if( !Cuda_ctrl::is_mesh_loaded() ){
        QMessageBox::information(this, "Error", "You must load a mesh before.");
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Load skeleton"),
                                                    "./resource/meshes",
                                                    tr("*.skel *.fbx") );
    if( fileName.size() != 0)
    {
        QFileInfo fi(fileName);
        QString ext = fi.suffix().toLower();

        if(ext == "fbx")
        {
            // Parse file
            Fbx_loader::Fbx_file loader( fileName.toStdString() );

            // Load into our data representation
            load_fbx_skeleton_anims( loader );
        }
        else if( ext == "skel")
        {
            Cuda_ctrl::_graph.load_from_file(fileName.toLatin1());
            Cuda_ctrl::_skeleton.load( *g_graph );
        }
        else
        {
            QMessageBox::information(this, "Error", "Unsupported file type: '"+ext+"'");
        }
    }
}

// -----------------------------------------------------------------------------

void Main_window_skin::on_actionSave_as_mesh_triggered()
{
    if( !Cuda_ctrl::is_mesh_loaded() ){
        QMessageBox::information(this, "Error", "No mesh to be saved.");
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save mesh"),
                                                    "./resource/meshes",
                                                    tr("*.off *obj") );

    if( fileName.size() != 0 )
    {
       QFileInfo fi(fileName);
       QString ext = fi.suffix().toLower();
       if(ext == "off")
           g_mesh->export_off(fileName.toLatin1(), false);
       else if( ext == "obj" )
       {
           Loader::Abs_mesh abs_mesh;
           g_mesh->save( abs_mesh );
           Obj_loader::Obj_file loader;
           loader.set_mesh( abs_mesh );
           loader.save_file( fileName.toStdString() );
       }
       else
           QMessageBox::information(this, "Error !", "unsupported ext: '"+ext+"' \n");
    }
}

// -----------------------------------------------------------------------------

void Main_window_skin::on_actionSave_ISM_triggered()
{
    if( !Cuda_ctrl::is_animesh_loaded() ){
        QMessageBox::information(this, "Error", "No animated mesh to save");
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save ism"),
                                                    "./resource/meshes",
                                                    tr("*.ism") );
    if( fileName.size() != 0)
        Cuda_ctrl::_anim_mesh->save_ism(fileName.toLatin1());
}

// -----------------------------------------------------------------------------

void Main_window_skin::on_actionLoad_ISM_triggered()
{
    if( !Cuda_ctrl::is_animesh_loaded() ){
        QMessageBox::information(this, "Error", "No animated mesh loaded");
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Load ism"),
                                                    "./resource/meshes",
                                                    tr("*.ism") );

    if( fileName.size() != 0)
        Cuda_ctrl::_anim_mesh->load_ism(fileName.toLatin1());
}

// -----------------------------------------------------------------------------

void Main_window_skin::on_actionLoad_weights_triggered()
{
    if( !Cuda_ctrl::is_animesh_loaded() ){
        QMessageBox::information(this, "Error", "No animated mesh loaded");
        return;
    }

    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Load skinning weights"),
                                                    "./resource/meshes",
                                                    tr("*.weights *.csv") );
    if( fileName.size() == 0)
        return;

    if(Cuda_ctrl::is_animesh_loaded())
        Cuda_ctrl::_anim_mesh->load_weights(fileName.toLatin1());
    else
        Cuda_ctrl::load_animesh_and_ssd_weights( fileName.toLatin1() );
}

// -----------------------------------------------------------------------------
void Main_window_skin::on_actionLoad_mesh_triggered()
{
}

void Main_window_skin::on_actionLoad_FBX_triggered()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                                    tr("Load from FBX"),
                                                    "./resource/meshes",
                                                    tr("*.fbx") );
    if( fileName.size() == 0)
        return;
    QString name = fileName.section('.',0,0);

    // Load mesh
    QString mesh_name = name;
    mesh_name.append(".fbx");
    if( !QFile::exists(mesh_name) )
    {
        QMessageBox::information(this, "Error !", "Can't' find "+name+"'.fbx'\n");
        return;
    }

    Fbx_loader::Fbx_file loader( mesh_name.toStdString() );
    load_fbx_mesh( loader );
    enable_mesh( true );

    load_fbx_skeleton_anims( loader );
}

