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
#ifndef _MAIN_WINDOW_SKIN_H_
#define _MAIN_WINDOW_SKIN_H_

#include <QComboBox>
#include <QSlider>
#include <QKeyEvent>
#include <map>

#include "fbx_loader.hpp"
#include "ui_main_window_skin.h"
#include "controller.hpp"

#include "OGL_widget_skin.hpp"

// FORWARD DEFS ----------------------------------------------------------------
// Forward def because of the interdependencie between MainWindow and
// OGL_viewports
class OGL_viewports_skin;
// END FORWARD DEFS ------------------------------------------------------------

/** @class MainWindow
  @brief The qt main window

  The window is designed through a '.ui' slots and signals are handle here.
  The class is implemented through severals main_window_xxx.cpp to seperate
  different parts of the window implementation wich is large
*/
class Main_window_skin : public QMainWindow, public Ui::MainWindow_skin {
    Q_OBJECT

public:
    Main_window_skin(QWidget *parent = 0);

    void choose_hrbf_samples(int bone_id);
    void choose_hrbf_samples_selected_bones();

    void update_ctrl_spin_boxes(const IBL::Ctrl_setup& shape);

    /// Set the controller for the current selected bone given the spinBox
    /// values
    void set_current_ctrl();

    /// Wether activate/or deactivate GUI related to mesh
    void enable_mesh(bool state);

    //void closeEvent(QCloseEvent*){ exit(0); }

    void keyPressEvent( QKeyEvent* event );

private:

    // -------------------------------------------------------------------------
    /// @name Widgets setup
    // -------------------------------------------------------------------------
    /// Connect toolbar_painting slots
    void setup_toolbar_painting();
    /// Populate 'comboB_operators'
    void setup_comboB_operators();
    /// Populate '_viewports'
    void setup_viewports();
    /// Hard coded window properties
    void setup_main_window();


    // -------------------------------------------------------------------------
    /// @name Tools
    // -------------------------------------------------------------------------

    /// Load skeleton and animation from a given fbx data structure
    bool load_fbx_skeleton_anims(const Fbx_loader::Fbx_file& loader);
    /// mesh from fbx loader
    void load_fbx_mesh( Fbx_loader::Fbx_file& loader);
    /// load skeleton '.skel'
    bool load_custom_skeleton(QString name);
    /// load ssd weights '.weights'
    bool load_custom_weights(QString name);

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    /// ComboBox to choose the type of selection (circle, square etc.)
    QComboBox* _selection_comboBox;

    /// The openGL viewports
    OGL_viewports_skin* _viewports;

public slots:
    // -------------------------------------------------------------------------
    /// @name MANUAL SLOTS
    // -------------------------------------------------------------------------

    /// Enable disable painting mode
    void paint_toggled(bool state);
    /// Update GUI relative to the active viewport
    void active_viewport(int id);

    void show_all_gizmo(bool checked);
    void set_gizmo_trans();
    void set_gizmo_rot();
    void set_gizmo_trackball();
    void set_gizmo_scale();
    void toggle_fitting(bool checked);


    // -------------------------------------------------------------------------
    /// @name AUTO SLOTS
    // -------------------------------------------------------------------------
    void on_actionLoad_mesh_triggered();
    void on_actionExit_triggered();
    void on_enable_smoothing_toggled(bool checked);
    void on_spinBox_smooth_iter_valueChanged(int );
    void on_horizontalSlider_sliderMoved(int position);
    void on_ssd_raio_toggled(bool checked);
    void on_actionLoad_skeleton_triggered();
    void on_spinBox_bulge_in_contact_force_valueChanged(double );
    void on_update_bulge_in_contact_released();
    void on_display_operator_toggled(bool checked);
    void on_display_controller_toggled(bool checked);
    void on_rbf_edition_toggled(bool checked);
    void on_actionSave_ISM_triggered();
    void on_actionLoad_ISM_triggered();
    void on_local_frame_toggled(bool checked);
    void on_reset_anim_released();
    void on_actionLoad_weights_triggered();
    void on_actionSave_as_mesh_triggered();
    void on_enable_partial_fit_toggled(bool checked);
    void on_spinBox_nb_step_fitting_valueChanged(int );
    void on_debug_show_normal_toggled(bool checked);
    void on_debug_show_gradient_toggled(bool checked);
    void on_spinB_smooth_force_a_valueChanged(double );
    void on_dual_quaternion_radio_toggled(bool checked);
    void on_implicit_skinning_checkBox_toggled(bool checked);
    void on_doubleSpinBox_valueChanged(double );
    void on_box_potential_pit_toggled(bool checked);
    void on_spinBox_diffuse_smoothing_weights_iter_valueChanged(int );
    void on_actionReloadShaders_triggered();
    void on_button_defects_point_cl_toggled(bool checked);
    void on_spinB_step_length_valueChanged(double );
    void on_checkBox_collsion_on_toggled(bool checked);
    void on_pushB_attached_skeleton_released();
    void on_pushB_set_rigid_weights_released();
    void on_pushB_diffuse_curr_weights_released();
    void on_pushB_diff_w_exp_released();
    void on_choose_hrbf_samples_released();
    void on_spinBox_valueChanged(int );
    void on_checkB_show_junction_toggled(bool checked);
    void on_dSpinB_min_dist_samples_valueChanged(double );
    void on_dSpinB_max_fold_valueChanged(double );
    void on_dSpinB_max_dist_joint_valueChanged(double );
    void on_dSpinB_max_dist_parent_valueChanged(double );
    void on_dSpinB_collision_depth_valueChanged(double );
    void on_dSpinB_ctrl_p0_x_valueChanged(double );
    void on_dSpinB_ctrl_p0_y_valueChanged(double );
    void on_dSpinB_ctrl_p1_x_valueChanged(double );
    void on_dSpinB_ctrl_p1_y_valueChanged(double );
    void on_dSpinB_ctrl_p2_x_valueChanged(double );
    void on_dSpinB_ctrl_p2_y_valueChanged(double );
    void on_dSpinB_ctrl_slope0_valueChanged(double );
    void on_dSpinB_ctrl_slope1_valueChanged(double );
    void on_comboB_operators_currentIndexChanged(int );
    void on_dSpinB_opening_value_valueChanged(double );
    void on_spinB_aperture_valueChanged(int );
    void on_checkB_cap_joint_toggled(bool checked);
    void on_checkB_capparent_toggled(bool checked);
    void on_dSpinB_hrbf_radius_valueChanged(double );
private slots:
    void on_checkBox_update_base_potential_toggled(bool checked);
    void on_pButton_compute_heat_difusion_released();

    void on_cBox_always_precompute_toggled(bool checked);
    void on_checkB_enable_smoothing_toggled(bool checked);
    void on_spinB_nb_iter_smooth1_valueChanged(int );
    void on_dSpinB_lambda_smooth1_valueChanged(double );
    void on_spinB_nb_iter_smooth2_valueChanged(int );
    void on_dSpinB_lambda_smooth2_valueChanged(double );
    void on_spinB_max_res_valueChanged(int );
    void on_checkB_align_with_normal_toggled(bool checked);
    void on_checkB_factor_siblings_toggled(bool checked);
    void on_pushB_empty_bone_released();

    void on_actionLoad_FBX_triggered();
    void on_actionSave_FBX_triggered();

    void on_pButton_add_caps_released();
    void on_pButton_supr_caps_released();
    void on_spinBox_2_valueChanged(int );
    void on_checkB_enable_raphson_toggled(bool checked);
    void on_color_smoothing_conservative_toggled(bool checked);
    void on_color_smoothing_laplacian_toggled(bool checked);
    void on_spinB_grid_res_valueChanged(int );
    void on_checkB_aa_bbox_clicked(bool checked);
};

#endif // _MAIN_WINDOW_H_
