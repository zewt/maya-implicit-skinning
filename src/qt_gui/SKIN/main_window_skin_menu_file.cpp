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

/*
    Implements what's concerning the file menu of the MainWindow class


*/

#include <QFileDialog>
#include <QColorDialog>
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

// TODO: to be deleted
extern Mesh* g_mesh;//
#include "graph.hpp"
extern Graph* g_graph;

///////////////////

using namespace Cuda_ctrl;

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
    if( QFile::exists(skel_name) )
    {
        Cuda_ctrl::_graph.load_from_file(skel_name.toLatin1());
        Cuda_ctrl::_skeleton.load( *g_graph );
        return true;
    }
    else
    {
        QMessageBox::information(this, "Error", "Can't' find "+name+".skel");
        return false;
    }
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

// -----------------------------------------------------------------------------

void Main_window_skin::on_actionSave_FBX_triggered()
{
    QMessageBox::information(this, "Error", "Not implemented yet");
    return;
    /*
    QString fileName = QFileDialog::getSaveFileName(this,
                                                    tr("Save to FBX"),
                                                    "./resource/meshes",
                                                    tr("*.fbx") );
    if( fileName.size() != 0){
    }
    */

}
