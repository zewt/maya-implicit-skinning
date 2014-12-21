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
#include <QString>

#include <cassert>
#include <algorithm>

#include "blending_lib/generator.hpp"
#include "vec3_cu.hpp"
#include "display_operator.hpp"
#include "SKIN/OGL_viewports_skin.hpp"
#include "cuda_ctrl.hpp"
#include "gl_mesh.hpp"
#include "loader.hpp"
#include "fbx_loader.hpp"
#include "conversions.hpp"


// TODO: to be deleted ////////////////
extern Mesh* g_mesh;
#include "graph.hpp"
extern Graph* g_graph;
#include "skeleton.hpp"
extern Skeleton* g_skel;

using namespace Cuda_ctrl;



#include "port_glew.h"

#include <QGLWidget>

class OGL_widget_skin_hidden : public QGLWidget {
public:
    OGL_widget_skin_hidden(QWidget *w): QGLWidget(w)
    {
        updateGL();
        makeCurrent();

        glewInit();
        int state = glewIsSupported("GL_VERSION_2_0 "
                                    "GL_VERSION_1_5 "
                                    "GL_ARB_vertex_buffer_object "
                                    "GL_ARB_pixel_buffer_object");
        if(!state) {
            fprintf(stderr, "Cannot initialize glew: required OpenGL extensions missing.");
            exit(-1);
        }
    
        setGeometry(0,0,0,0);
        hide();

        assert(isValid());

        // Initialize cuda context
        std::vector<Blending_env::Op_t> op;
    //    op.push_back( Blending_env::B_TCH );
        op.push_back( Blending_env::B_D  );
        op.push_back( Blending_env::U_OH );
        op.push_back( Blending_env::C_D  );

    //    for (int i=(int)Blending_env::BINARY_3D_OPERATOR_BEGIN+1; i<(int)Blending_env::BINARY_3D_OPERATOR_END; ++i)
    //        op.push_back( Blending_env::Op_t(i) );

        Cuda_ctrl::cuda_start( op );
        Cuda_ctrl::init_opengl_cuda();
    }
};

/*
void OGL_viewports_skin::updateGL()
{
    using namespace Cuda_ctrl;

    if(_anim_mesh == NULL)
        return;
    // Transform HRBF samples for display and selection
    _anim_mesh->transform_samples();
    // Animate the mesh :
    _anim_mesh->deform_mesh();
}
*/









// -----------------------------------------------------------------------------

Main_window_skin::Main_window_skin(QWidget *parent) :
    QMainWindow(parent)
{
    setupUi(this);

    _hidden = new OGL_widget_skin_hidden(viewports_frame);

//    _viewports = new OGL_viewports_skin(viewports_frame, this);
//    viewports_frame->layout()->addWidget(_viewports);

    resize(1300, 800);

    // HACK: Because blending_env initialize to elbow too ...
    IBL::Ctrl_setup shape = IBL::Shape::elbow();

    Fbx_loader::init();

//    Skeleton_env::set_grid_res(0, 20); // FIXME: remove hardcoded skeleton ID

//    Cuda_ctrl::_debug._smooth_mesh = true;
//    Cuda_ctrl::_debug._smooth1_iter = 7;
//    Cuda_ctrl::_debug._smooth1_force = 1.0f;
//    Cuda_ctrl::_debug._smooth2_iter = 1;
//    Cuda_ctrl::_debug._smooth2_force= 0.5f;
//    Cuda_ctrl::_anim_mesh->set_nb_iter_smooting(7); // 1 ... 300
}

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

        break;
    }

    case 1:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_ad_hoc
                (bone_id,
                 -0.02f, // dSpinB_max_dist_joint->value(),
                 -0.02f, // dSpinB_max_dist_parent->value(),
                 0, // dSpinB_min_dist_samples->value(), Minimal distance between two HRBF sample
                 0); // dSpinB_max_fold->value() );

        break;
    }

    case 2:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_gael( bone_id );
        break;
    }
    }
}

void Main_window_skin::choose_hrbf_samples_selected_bones()
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    for(unsigned i = 0; i < set.size(); i++)
        choose_hrbf_samples(set[i]);

    Cuda_ctrl::_anim_mesh->update_base_potential();
}

void Main_window_skin::set_current_ctrl()
{
    IBL::float2 p0;
    p0.x = 0.20f; // -3 ... +3.0, step 0.05
    p0.y = 1.00f; //    ... +1.0, step 0.05

    IBL::float2 p1;
    p1.x = 0.70f; // dSpinB_ctrl_p1_x->value();
    p1.y = 0.43f; // dSpinB_ctrl_p1_y->value();

    IBL::float2 p2;
    p2.x = 1.20f; // dSpinB_ctrl_p2_x->value();
    p2.y = 1.00f; // dSpinB_ctrl_p2_y->value();

    IBL::float2 s;
    s.x = 1.0f; // dSpinB_ctrl_slope0->value();
    s.y = 1.0f; // dSpinB_ctrl_slope1->value();

    IBL::Ctrl_setup shape(p0, p1, p2, s.x, s.y);

    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    if(set.size() > 0)
        for(unsigned i = 0; i < set.size(); i++)
            _skeleton.set_joint_controller(set[i], shape);
    else
        _operators.set_global_controller(shape);

    _anim_mesh->update_base_potential();
}

void Main_window_skin::toggle_fitting(bool checked){
    Cuda_ctrl::_anim_mesh->set_implicit_skinning(checked);
}

void Main_window_skin::on_actionExit_triggered()
{
    exit(0);
}

// SMOOTHING SLOTS =============================================================

void Main_window_skin::on_enable_smoothing_toggled(bool checked){
    Cuda_ctrl::_anim_mesh->set_do_smoothing(checked);
}

void Main_window_skin::on_spinB_smooth_force_a_valueChanged(double val){
    Cuda_ctrl::_anim_mesh->set_smooth_force_a(val);
}

// END SMOOTHING SLOTS =========================================================

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











void Main_window_skin::load_fbx_mesh( Fbx_loader::Fbx_file& loader)
{
    Mesh* ptr_mesh = new Mesh();
    Loader::Abs_mesh mesh;
    loader.get_mesh( mesh );
    if(mesh._vertices.size() == 0)
        return;

    ptr_mesh->load( mesh, loader._path);
    Cuda_ctrl::load_mesh( ptr_mesh );
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
    if( !Cuda_ctrl::is_mesh_loaded() )
        return;

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
    }
}

// -----------------------------------------------------------------------------

void Main_window_skin::on_actionSave_as_mesh_triggered()
{
    if( !Cuda_ctrl::is_mesh_loaded() )
        return;

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save mesh"),
                                                    "./resource/meshes",
                                                    tr("*.off") );

    if( fileName.size() != 0 )
    {
       QFileInfo fi(fileName);
       QString ext = fi.suffix().toLower();
       if(ext == "off")
           g_mesh->export_off(fileName.toLatin1(), false);
    }
}

// -----------------------------------------------------------------------------

void Main_window_skin::on_actionSave_ISM_triggered()
{
    if( !Cuda_ctrl::is_animesh_loaded() )
        return;

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
    if( !Cuda_ctrl::is_animesh_loaded() )
        return;

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
    if( !Cuda_ctrl::is_animesh_loaded() )
        return;

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
        return;

    Fbx_loader::Fbx_file loader( mesh_name.toStdString() );
    load_fbx_mesh( loader );

    load_fbx_skeleton_anims( loader );
}

