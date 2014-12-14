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
#ifndef PRESET_CONTROLERS_HPP__
#define PRESET_CONTROLERS_HPP__

#include <QPushButton>
#include <QToolButton>
#include "ui_preset_controllers.h"

#include "blending_lib/controller.hpp"
#include "skeleton_ctrl.hpp"
#include "animated_mesh_ctrl.hpp"
#include "operators_ctrl.hpp"


namespace Cuda_ctrl {
/// @see cuda_ctrl.cu
extern Animated_mesh_ctrl* _anim_mesh;
extern Skeleton_ctrl _skeleton;
extern Operators_ctrl _operators;
}

class Preset_ctrl;

// =============================================================================
class Button_ctrl : public QWidget{
    Q_OBJECT
public:
    Button_ctrl(const QString& name,
                const IBL::Ctrl_setup& s,
                Preset_ctrl* parent);

    ~Button_ctrl();

    QString get_name();

signals:
    /// Triggered when the remove button is pushed
    void remove(Button_ctrl*);

private slots:
    void pushed();
    void remove_button();

private:
    IBL::Ctrl_setup _shape;
    QPushButton*    _button_preset;
    QToolButton*    _button_remove;
    QHBoxLayout*    _hor_layout;
};
// END Button_ctrl =============================================================



// =============================================================================
/** @class Preset_ctrl
  @brief A custom widget to handle presets for the controller shapes

  This widget provides means to read and write text files of controlers
  charateristics. It dynamicaly adds buttons to the GUI to set the controller
  shape given the list of file presets inside a folder

  @see Button_ctrl
*/
class Preset_ctrl : public QWidget, public Ui::PresetControllers
{
    Q_OBJECT
public:

    Preset_ctrl(QWidget *parent = 0);
    ~Preset_ctrl();

signals:
    /// Triggered when a preset button has been pushed
    void preset_pushed();
private:
    Button_ctrl* read_preset(QString filename);
    void write_preset(const IBL::Ctrl_setup& s, const QString& filename);
    void load_presets();

    QVBoxLayout* _layout;

private slots:
    void on_pushB_save_released();
    void on_pushB_load_released();

    /// Preset button has been pushed
    void button_pushed();

    void remove_button(Button_ctrl* to_delete);
};
// END Preset_ctrl =============================================================

#endif // PRESET_CONTROLERS_HPP__
