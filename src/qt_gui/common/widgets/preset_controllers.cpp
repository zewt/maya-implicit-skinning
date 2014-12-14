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
#include "common/widgets/preset_controllers.hpp"

#include <QDir>
#include <QFileInfo>
#include <QFileDialog>
#include <iostream>
#include <fstream>

#include "common/tools/popup_ok_cancel.hpp"

/// @see globals.cpp
extern std::string g_config_dir;
extern std::string g_icons_dir;


// Button_ctrl =================================================================

Button_ctrl::Button_ctrl(const QString& name,
            const IBL::Ctrl_setup& s,
            Preset_ctrl* parent):
    QWidget(parent),
    _shape(s)
{
    _hor_layout = new QHBoxLayout(this);

    //this->setSpacing(1);
    _hor_layout->setContentsMargins(0, 0, 0, 0);

    _button_preset = new QPushButton(this);

    _button_remove = new QToolButton(this);
    QIcon icon_rec(QString(g_icons_dir.c_str())+"/delete.png");
    _button_remove->setObjectName("remove_button");
    _button_remove->setIcon(icon_rec);
    _button_remove->setIconSize(QSize(16, 16));

    _hor_layout->addWidget(_button_preset);
    _hor_layout->addWidget(_button_remove);

    _button_preset->setText(name);
    connect(_button_preset, SIGNAL(released()), this  , SLOT(pushed())       );
    connect(_button_preset, SIGNAL(released()), parent, SLOT(button_pushed()));

    connect(_button_remove, SIGNAL(released()), this, SLOT(remove_button()));

    connect(this, SIGNAL(remove(Button_ctrl*)), parent, SLOT(remove_button(Button_ctrl*)));
}

// -----------------------------------------------------------------------------

Button_ctrl::~Button_ctrl(){
}

// -----------------------------------------------------------------------------

void Button_ctrl::pushed()
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

    if(set.size() > 0){
        for(unsigned i = 0; i < set.size(); i++)
            Cuda_ctrl::_skeleton.set_joint_controller(set[i], _shape);
    }else{
        Cuda_ctrl::_operators.set_global_controller(_shape);
    }

    if (Cuda_ctrl::_anim_mesh != 0)
        Cuda_ctrl::_anim_mesh->update_base_potential();
}

// -----------------------------------------------------------------------------

void Button_ctrl::remove_button()
{
    emit remove(this);
}

// -----------------------------------------------------------------------------

QString Button_ctrl::get_name()
{
    return _button_preset->text();
}

// END Button_ctrl =============================================================

Preset_ctrl::Preset_ctrl(QWidget *parent) :
    QWidget(parent),
    _layout(0)
{
    setupUi(this);
    load_presets();
}

// -----------------------------------------------------------------------------

Preset_ctrl::~Preset_ctrl()
{
}

// -----------------------------------------------------------------------------

void Preset_ctrl::load_presets()
{
    // Remove all buttons
    verticalLayout->removeWidget(widget_presets);
    widget_presets->close();
    delete widget_presets;

    widget_presets = new QWidget(this);
    widget_presets->setObjectName(QString::fromUtf8("widget_presets"));
    verticalLayout->addWidget(widget_presets);

    _layout = new QVBoxLayout(widget_presets);
    _layout->setSpacing(0);
    _layout->setContentsMargins(0, 0, 0, 0);


    // Add buttons for each preset file
    QDir dir(QString(g_config_dir.c_str())+"/presets");

    QStringList filters;
    filters << "*.ctr"; /*<< "*.cxx" << "*.cc"*/;
    QDir::Filters f(QDir::Readable | QDir::NoDotAndDotDot | QDir::Files);
    QFileInfoList list = dir.entryInfoList(filters, f);

    for(int i = 0; i < list.size(); i++)
    {
        Button_ctrl* b = read_preset( list[i].absoluteFilePath() );
        _layout->addWidget(b);
    }
}

// -----------------------------------------------------------------------------

Button_ctrl* Preset_ctrl::read_preset(QString filename)
{
    std::ifstream file(filename.toAscii());

    std::string dummy;

    IBL::float2 p0;
    file >> dummy >> p0.x >> dummy >> p0.y >> dummy;
    IBL::float2 p1;
    file >> dummy >> p1.x >> dummy >> p1.y >> dummy;
    IBL::float2 p2;
    file >> dummy >> p2.x >> dummy >> p2.y >> dummy;

    float s0, s1;
    file >> dummy >> s0 >> dummy >> s1 >> dummy;

    IBL::Ctrl_setup s(p0, p1, p2, s0, s1);

    QFileInfo infos(filename);
    Button_ctrl* button = new Button_ctrl(infos.baseName(), s, this);

    file.close();
    return button;
}

// -----------------------------------------------------------------------------

void Preset_ctrl::write_preset(const IBL::Ctrl_setup& s,
                               const QString& filename)
{
    std::ofstream file(filename.toAscii(), std::ios::trunc);

    file << "p0( " << s.p0().x << " , " << s.p0().y << " )" << std::endl;
    file << "p1( " << s.p1().x << " , " << s.p1().y << " )" << std::endl;
    file << "p2( " << s.p2().x << " , " << s.p2().y << " )" << std::endl;
    file << "s( "  << s.s0()   << " , " << s.s1()   << " )" << std::endl;

    file.close();
}

// -----------------------------------------------------------------------------

void Preset_ctrl::on_pushB_save_released()
{
    const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();


        QString name = QFileDialog::getSaveFileName(this,
                                                    tr("Load preset"),
                                                    QString(g_config_dir.c_str())+"/presets",
                                                    tr("*.ctr") );

        if(name == "" ) return;

        QFileInfo infos(name);

        name = infos.suffix() == "" ? name + ".ctr" : name;

        IBL::Ctrl_setup shape;
        if(set.size() > 0)
            shape = Cuda_ctrl::_skeleton.get_joint_controller(set[set.size()-1]);
        else
            shape = Cuda_ctrl::_operators.get_global_controller();

        write_preset(shape, name);

        // Reload buttons
        load_presets();
}

// -----------------------------------------------------------------------------

void Preset_ctrl::on_pushB_load_released()
{
    QString name = QFileDialog::getOpenFileName(this,
                                                tr("Load preset"),
                                                QString(g_config_dir.c_str())+"/presets",
                                                tr("*.ctr") );
    Button_ctrl* b = read_preset( name );
    _layout->addWidget(b);
}

// -----------------------------------------------------------------------------

void Preset_ctrl::button_pushed()
{
    emit preset_pushed();
}

// -----------------------------------------------------------------------------

void Preset_ctrl::remove_button(Button_ctrl* to_delete)
{
    QString name = QString(g_config_dir.c_str())+"/presets/"+to_delete->get_name()+".ctr";
    Diag_ok_cancel diag("Remove preset", "\nRemove the preset '"+name+"' ?\n", this);

    if( diag.exec() )
    {
        QFile::remove(name);
        load_presets();
    }
}

// -----------------------------------------------------------------------------
