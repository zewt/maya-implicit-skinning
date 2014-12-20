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
    setup_toolbar();
    setup_toolbar_frame();
    setup_comboB_operators();
    set_up_settings_raytracing();
    setup_main_window();
    //--------------

    QObject::connect(widget_ctrl_presets, SIGNAL( preset_pushed()       ),
                     this               , SLOT  ( ctrl_presets_pushed() ));

    // HACK: Because blending_env initialize to elbow too ...
    IBL::Ctrl_setup shape = IBL::Shape::elbow();
    update_ctrl_spin_boxes(shape);

    // Desactivate GUI parts
    enable_animesh( false );
    enable_mesh   ( false );
}

// -----------------------------------------------------------------------------

void Main_window_skin::setup_toolbar()
{
    // Connect toolButtons for selection
    Widget_selection* select = toolBar->_wgt_select;
    QObject::connect(select->toolB_select_point, SIGNAL(pressed()), this, SLOT(selection_toolb_point()));
    QObject::connect(select->toolB_select_circle, SIGNAL(pressed()), this, SLOT(selection_toolb_circle()));
    QObject::connect(select->toolB_select_square, SIGNAL(pressed()), this, SLOT(selection_toolb_square()));

    Widget_viewports* view = toolBar->_wgt_viewport;
    QObject::connect(view->toolB_single, SIGNAL(pressed()), this, SLOT(viewport_toolb_single()));
    QObject::connect(view->toolB_doubleV, SIGNAL(pressed()), this, SLOT(viewport_toolb_doubleV()));
    QObject::connect(view->toolB_doubleH, SIGNAL(pressed()), this, SLOT(viewport_toolb_doubleH()));
    QObject::connect(view->toolB_four, SIGNAL(pressed()), this, SLOT(viewport_toolb_four()));

    // Connect toolButtons for the rendering mode
    Widget_render_mode* render = toolBar->_wgt_rd_mode;
    QObject::connect(render->toolB_wire_transc, SIGNAL(pressed()), this, SLOT(rd_mode_toolb_wire_transc()));
    QObject::connect(render->toolB_wire, SIGNAL(pressed()), this, SLOT(rd_mode_toolb_wire()));
    QObject::connect(render->toolB_solid, SIGNAL(pressed()), this, SLOT(rd_mode_toolb_solid()));
    QObject::connect(render->toolB_tex, SIGNAL(pressed()), this, SLOT(rd_mode_toolb_tex()));

    // Fitting buton
    QObject::connect(toolBar->_wgt_fit->toolB_fitting, SIGNAL(toggled(bool)), this, SLOT(toggle_fitting(bool)));

    // Connect combo box for the pivot mode
    QObject::connect(toolBar->_pivot_comboBox, SIGNAL(currentIndexChanged(int)),
                     this           , SLOT(pivot_comboBox_currentIndexChanged(int)));
}

// -----------------------------------------------------------------------------

void Main_window_skin::setup_toolbar_frame()
{
    QObject::connect(toolBar_frame, SIGNAL(update_gl()),
                     this         , SLOT(update_viewports()));
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

void Main_window_skin::set_up_settings_raytracing()
{
    QObject::connect(settings_raytracing, SIGNAL(update_viewports()), this, SLOT(update_viewports()));
    QObject::connect(settings_raytracing, SIGNAL(update_raytracing()), this, SLOT(update_raytracing()));
    QObject::connect(settings_raytracing->enable_raytracing, SIGNAL(toggled(bool)),
                     this, SLOT(settings_raytracing_on_enable_raytracing_toggled(bool)));
    QObject::connect(settings_raytracing->potential_plane_pos, SIGNAL(released()),
                     this, SLOT(settings_raytracing_on_potential_plane_pos_released()));
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
    }break;
    }

    update_raytracing();
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

void Main_window_skin::update_viewports()
{
    _viewports->updateGL();
}

// -----------------------------------------------------------------------------

void Main_window_skin::update_raytracing()
{
    Cuda_ctrl::_display._raytrace_again = true;
}

// -----------------------------------------------------------------------------

void Main_window_skin::paint_toggled(bool state)
{
    if(state){
        toolBar->_wgt_select->toolB_select_circle->setChecked(true);
        // trigger corresponding slot
        selection_toolb_circle();
        toolBar->_wgt_select->setEnabled( false );
    }
    else
    {
        toolBar->_wgt_select->setEnabled( true );
    }
}

// MANUAL SLOTS ################################################################

void Main_window_skin::selection_toolb_point()
{
    Vec_viewports& list = _viewports->get_viewports();

    for(unsigned i = 0; i < list.size(); i++)
    {
        list[i]->setMouseTracking(false);
        list[i]->set_selection(EOGL_widget::MOUSE);
    }
    update_viewports();
}

void Main_window_skin::selection_toolb_circle()
{
    Vec_viewports& list = _viewports->get_viewports();

    for(unsigned i = 0; i < list.size(); i++)
    {
        list[i]->setMouseTracking(false);

        list[i]->set_selection(EOGL_widget::CIRCLE);
        list[i]->setMouseTracking(true);
    }
    update_viewports();
}

void Main_window_skin::selection_toolb_square(){

}

void Main_window_skin::viewport_toolb_single(){
    _viewports->set_viewports_layout(OGL_viewports_skin::SINGLE);
    _viewports->updateGL();
}

void Main_window_skin::viewport_toolb_doubleV(){
    _viewports->set_viewports_layout(OGL_viewports_skin::VDOUBLE);
    _viewports->updateGL();
}

void Main_window_skin::viewport_toolb_doubleH(){
    _viewports->set_viewports_layout(OGL_viewports_skin::HDOUBLE);
    _viewports->updateGL();
}

void Main_window_skin::viewport_toolb_four(){
    _viewports->set_viewports_layout(OGL_viewports_skin::FOUR);
    _viewports->updateGL();
}

void Main_window_skin::rd_mode_toolb_tex()
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    wgl->set_phong( true );
    wgl->set_textures( true );
    update_viewports();
}

void Main_window_skin::rd_mode_toolb_solid()
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    wgl->set_phong( true );
    wgl->set_textures( false );
    update_viewports();
}

void Main_window_skin::rd_mode_toolb_wire()
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    wgl->set_phong( false );
    Cuda_ctrl::_display.set_transparency_factor( 1.f);
    update_viewports();
}

void Main_window_skin::rd_mode_toolb_wire_transc()
{
    OGL_widget_skin* wgl = _viewports->active_viewport();
    Cuda_ctrl::_display.set_transparency_factor( 0.5f );
    wgl->set_phong( false );
    update_viewports();
}

void Main_window_skin::show_all_gizmo(bool checked) {}
void Main_window_skin::set_gizmo_trans() {}
void Main_window_skin::set_gizmo_rot() {}
void Main_window_skin::set_gizmo_trackball() {}
void Main_window_skin::set_gizmo_scale() {}

void Main_window_skin::toggle_fitting(bool checked){
    Cuda_ctrl::_anim_mesh->set_implicit_skinning(checked);
    update_viewports();
}

void Main_window_skin::pivot_comboBox_currentIndexChanged(int idx)
{
    int val = toolBar->_pivot_comboBox->itemData( idx ).toInt();
    _viewports->set_pivot_mode((EOGL_widget::Pivot_t)val);
}

void Main_window_skin::ctrl_presets_pushed()
{
    IBL::Ctrl_setup shape;
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    if(set.size() > 0)
        for(unsigned i = 0; i < set.size(); i++){
            shape = _skeleton.get_joint_controller( set[i] );
            update_ctrl_spin_boxes(shape);
        }
    else{
        shape = _operators.get_global_controller();
        update_ctrl_spin_boxes(shape);
    }

    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

void Main_window_skin::active_viewport(int id)
{
    static int id_prev = -1;
    // Update necessary only if the active viewport is changed
    if(id_prev == id) return;

    OGL_widget_skin* wgl = _viewports->active_viewport();

    // update the pannel buttons:
    settings_raytracing->enable_raytracing->setChecked( wgl->raytrace() );

    id_prev = id;
}

// AUTOMATIC SLOTS #############################################################

void Main_window_skin::on_actionExit_triggered()
{
    exit(0);
}

// SMOOTHING SLOTS =============================================================

void Main_window_skin::on_enable_smoothing_toggled(bool checked){
    Cuda_ctrl::_anim_mesh->set_do_smoothing(checked);
    update_viewports();
}

void Main_window_skin::on_spinBox_smooth_iter_valueChanged(int val){
    Cuda_ctrl::_anim_mesh->set_nb_iter_smooting(val);
    update_viewports();
}

void Main_window_skin::on_spinB_smooth_force_a_valueChanged(double val){
    Cuda_ctrl::_anim_mesh->set_smooth_force_a(val);
    update_viewports();
}

void Main_window_skin::on_checkB_enable_smoothing_toggled(bool checked)
{
    Cuda_ctrl::_debug._smooth_mesh = checked;
    update_viewports();
}

void Main_window_skin::on_spinB_nb_iter_smooth1_valueChanged(int val)
{
    Cuda_ctrl::_debug._smooth1_iter = val;
    update_viewports();
}

void Main_window_skin::on_dSpinB_lambda_smooth1_valueChanged(double val)
{
    Cuda_ctrl::_debug._smooth1_force = val;
    update_viewports();
}

void Main_window_skin::on_spinB_nb_iter_smooth2_valueChanged(int val)
{
    Cuda_ctrl::_debug._smooth2_iter = val;
    update_viewports();
}

void Main_window_skin::on_dSpinB_lambda_smooth2_valueChanged(double val)
{
    Cuda_ctrl::_debug._smooth2_force= val;
    update_viewports();
}

// END SMOOTHING SLOTS =========================================================

void Main_window_skin::on_horizontalSlider_sliderMoved(int position)
{
    Cuda_ctrl::_display.set_transparency_factor( (float)position/100.f );
    update_viewports();
}

void Main_window_skin::on_ssd_raio_toggled(bool checked)
{
    if(checked)
    {
        Cuda_ctrl::_anim_mesh->do_ssd_skinning();
        update_viewports();
    }
}

void Main_window_skin::on_dual_quaternion_radio_toggled(bool checked)
{
    if(checked)
    {
        Cuda_ctrl::_anim_mesh->do_dual_quat_skinning();
        update_viewports();
    }
}

void Main_window_skin::on_implicit_skinning_checkBox_toggled(bool checked)
{
    Cuda_ctrl::_anim_mesh->set_implicit_skinning(checked);
    update_viewports();
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
    update_viewports();
}

void Main_window_skin::on_update_bulge_in_contact_released()
{
    Cuda_ctrl::_operators.update_bulge();
    update_viewports();
}

// END BLENDING SLOTS ==========================================================

// RBF EDITION SLOTS ===========================================================

void Main_window_skin::on_rbf_edition_toggled(bool checked)
{
    Vec_viewports& list = _viewports->get_viewports();

    for(unsigned i = 0; i < list.size(); i++)
    {
        if(checked) list[i]->set_io(EOGL_widget::RBF);
        else        list[i]->set_io(EOGL_widget::MESH_EDIT);
    }

    Cuda_ctrl::_display._edit_hrbf_samples = checked;

    update_viewports();
}

void Main_window_skin::on_local_frame_toggled(bool){
    update_viewports();
}

void Main_window_skin::on_checkB_align_with_normal_toggled(bool){
    update_viewports();
}

void Main_window_skin::on_checkB_factor_siblings_toggled(bool checked)
{
    Cuda_ctrl::_anim_mesh->set_factor_siblings( checked );
}

// END RBF EDITION SLOTS =======================================================

// HELP MENU ===================================================================

void Main_window_skin::on_actionShortcuts_triggered()
{
    QMessageBox::information(this,
                             "Shortcuts",
                             "Tabulation : switch to the edit mode for point selection\n\n"
                             "CTRL-A : select all skeleton bones\n\n"
                             "CTRL : add to selection with a left click \n\n"
                             "SHIFT : remove from selection with a left click \n\n"
                             "'r' : raytrace the selected bone or the last two selected bones.\n"
                             "if no bones are selected the entire skeleton is rendered\n\n"
                             "'BACKSPACE' : Skeleton position to rest pose \n\n"
                             "'o' : switch between SSD and implicit skinning\n\n"
                             "'F1' : enable/disable mesh smoothing\n\n"
                             "'j' : When holding moves the joint with a left click.\n"
                             "Warning : moving the joint do not recompute automaticaly SSD weights !\n\n"
                             "'t' : hide or show the mesh\n\n"
                             "'T' : switch between phong and the transparent mode\n\n"
                             "space : change the bone's type (multiple bones can be selected at the same time)\n\n"
                             "'u' : update the base potential of vertices in the dress pose.\n"
                             "This should be automatic when implicit surfaces changes or blends differently.\n"
                             "But when it doesn't work the shortcut is useful to manually overide the bug...\n\n"
                             "'g' : Draw a grid\n\n"
                             "'5' : switch between perspective view and orthogonal view \n\n"
                             "'8', '4', '6', '1', '2', '3' : Different points of views : top, left, right etc.\n\n");
}

void Main_window_skin::on_actionAbout_triggered()
{
    QMessageBox::information(this,
                             "Authors",
                             "Coding, conception && research :\n"
                             "(alphabetical order of first names)\n"
                             "Damien Rohmer, Florian Canezin, Gael Guennebaud, Loic Barthe, Olivier Gourmel, Rodolphe Vaillant\n"
                             "Logo, theme && icons\n"
                             "Judith Belin\n");
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
    update_viewports();
}

void Main_window_skin::on_enable_partial_fit_toggled(bool checked)
{
    Cuda_ctrl::_debug._do_partial_fit = checked;
    update_viewports();
}

void Main_window_skin::on_spinBox_nb_step_fitting_valueChanged(int val)
{
    Cuda_ctrl::_debug._nb_step = val;
    update_viewports();
}

void Main_window_skin::on_debug_show_normal_toggled(bool checked)
{
    Cuda_ctrl::_debug._show_normals = checked;
    update_viewports();
}

void Main_window_skin::on_debug_show_gradient_toggled(bool checked)
{
    Cuda_ctrl::_debug._show_gradient = checked;
    update_viewports();
}

void Main_window_skin::on_actionColors_triggered()
{
}

void Main_window_skin::on_doubleSpinBox_valueChanged(double val)
{
    Cuda_ctrl::_debug._collision_threshold = val;
    update_viewports();
}

void Main_window_skin::on_box_potential_pit_toggled(bool checked)
{
    Cuda_ctrl::_debug._potential_pit = checked;
    update_viewports();
}

void Main_window_skin::on_spinBox_diffuse_smoothing_weights_iter_valueChanged(int val)
{
    Cuda_ctrl::_anim_mesh->set_smoothing_weights_diffusion_iter(val);
    update_viewports();
}

void Main_window_skin::on_actionReloadShaders_triggered()
{
    Cuda_ctrl::reload_shaders();
    update_viewports();
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
        update_viewports();
    }
}

void Main_window_skin::on_spinB_step_length_valueChanged(double val)
{
    Cuda_ctrl::_debug._step_length = val;
    update_viewports();
}

void Main_window_skin::on_checkBox_collsion_on_toggled(bool checked)
{
    Cuda_ctrl::_debug._fit_on_all_bones = checked;
    update_viewports();
}

void Main_window_skin::on_pushB_attached_skeleton_released()
{
    if( g_graph != 0 && g_graph->_vertices.size() > 0 &&
        Cuda_ctrl::is_mesh_loaded() )
    {

        Cuda_ctrl::_skeleton.load( *g_graph );
        Cuda_ctrl::load_animesh();
        _viewports->set_io(EOGL_widget::MESH_EDIT);
        enable_animesh( true );
        update_viewports();
    }
}

void Main_window_skin::on_pushB_set_rigid_weights_released()
{
    Cuda_ctrl::_anim_mesh->init_rigid_ssd_weights();
    update_viewports();
}

void Main_window_skin::on_pushB_diffuse_curr_weights_released()
{
    Cuda_ctrl::_anim_mesh->topology_diffuse_ssd_weights(dSpinB_diff_w_alpha->value(), spinB_diff_w_nb_iter->value());
    update_viewports();
}

void Main_window_skin::on_pushB_diff_w_exp_released()
{
    Cuda_ctrl::_anim_mesh->geodesic_diffuse_ssd_weights(dSpinB_diff_w_alpha_exp->value(), spinB_auto_w_nb_iter_exp->value());
    update_viewports();
}

void Main_window_skin::on_choose_hrbf_samples_released()
{
    choose_hrbf_samples_selected_bones();
    update_viewports();
}

void Main_window_skin::on_checkB_show_junction_toggled(bool checked)
{
    Cuda_ctrl::_display._junction_spheres = checked;
    update_viewports();
}

void Main_window_skin::on_dSpinB_min_dist_samples_valueChanged(double )
{
    if(checkB_auto_sample->isChecked())
    {
        choose_hrbf_samples_selected_bones();
        update_viewports();
    }
}

void Main_window_skin::on_dSpinB_max_fold_valueChanged(double )
{
    if(checkB_auto_sample->isChecked())
    {
        choose_hrbf_samples_selected_bones();
        update_viewports();
    }
}

void Main_window_skin::on_dSpinB_max_dist_joint_valueChanged(double )
{
    if(checkB_auto_sample->isChecked())
    {
        choose_hrbf_samples_selected_bones();
        update_viewports();
    }
}

void Main_window_skin::on_dSpinB_max_dist_parent_valueChanged(double )
{
    if(checkB_auto_sample->isChecked())
    {
        choose_hrbf_samples_selected_bones();
        update_viewports();
    }
}

void Main_window_skin::on_dSpinB_collision_depth_valueChanged(double val)
{
    Cuda_ctrl::_debug._collision_depth = val;
    update_viewports();
}

// CONTROLLER SPINBOXES ========================================================

void Main_window_skin::on_dSpinB_ctrl_p0_x_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

void Main_window_skin::on_dSpinB_ctrl_p0_y_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

void Main_window_skin::on_dSpinB_ctrl_p1_x_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

void Main_window_skin::on_dSpinB_ctrl_p1_y_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

void Main_window_skin::on_dSpinB_ctrl_p2_x_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

void Main_window_skin::on_dSpinB_ctrl_p2_y_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

void Main_window_skin::on_dSpinB_ctrl_slope0_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

void Main_window_skin::on_dSpinB_ctrl_slope1_valueChanged(double )
{
    set_current_ctrl();
    Cuda_ctrl::_display._raytrace_again = true;
    update_viewports();
}

// END CONTROLLER SPINBOXES ====================================================

void Main_window_skin::on_checkB_cap_joint_toggled(bool checked)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->set_jcap(set[i], checked);

    update_viewports();
}

void Main_window_skin::on_checkB_capparent_toggled(bool checked)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->set_pcap(set[i], checked);

    update_viewports();
}

void Main_window_skin::on_dSpinB_hrbf_radius_valueChanged(double val)
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();
    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->set_hrbf_radius(set[i], val);

    Cuda_ctrl::_anim_mesh->update_base_potential();

    update_raytracing();
    update_viewports();
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
        update_viewports();
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
    update_raytracing();
    update_viewports();
}


void Main_window_skin::on_pushB_empty_bone_released()
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();
    for(unsigned i = 0; i < set.size(); i++)
        Cuda_ctrl::_anim_mesh->empty_samples( set[i] );

    update_raytracing();
    update_viewports();
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
    update_raytracing();
    update_viewports();
}

void Main_window_skin::on_pButton_supr_caps_released()
{
    set_caps_selected_joints(false);
    update_raytracing();
    update_viewports();
}

void Main_window_skin::on_spinBox_2_valueChanged(int val)
{
    Cuda_ctrl::_debug._slope_smooth_weight = val % 2 == 0 ? val : val + 1;
    update_viewports();
}

void Main_window_skin::on_checkB_enable_raphson_toggled(bool checked)
{
    Cuda_ctrl::_debug._raphson = checked;
    update_viewports();
}

void Main_window_skin::on_color_smoothing_conservative_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::ANIM_SMOOTH_CONSERVATIVE);
        update_viewports();
    }
}

void Main_window_skin::on_color_smoothing_laplacian_toggled(bool checked)
{
    if(checked){
        Cuda_ctrl::_anim_mesh->color_type(EAnimesh::ANIM_SMOOTH_LAPLACIAN);
        update_viewports();
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
    update_viewports();
}

void Main_window_skin::on_checkB_aa_bbox_clicked(bool checked)
{
    Cuda_ctrl::_display._aa_bbox= checked;
    update_viewports();
}
