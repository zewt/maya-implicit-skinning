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
#ifndef IO_SKELETON_HPP__
#define IO_SKELETON_HPP__

#include "SKIN/IO_interface_skin.hpp"
#include "vec2_cu.hpp"

#include <QMenu>
#include <map>


/** @brief Handle mouse and keys for skeleton animation

  @see IO_interface
*/

class IO_skeleton : public IO_interface_skin {
public:

    IO_skeleton(OGL_widget_skin* gl_widget) :
        IO_interface_skin(gl_widget),
        _janim_on(true),
        _mouse_z(0.f),
        _joint(-1),
        _last_vertex(-1),
        _nb_selected_vert(0),
        _move_joint_mode(false),
        _selection_hysteresis(8.f)
    {
        using namespace Cuda_ctrl;

        _gl_widget->set_draw_skeleton( true );

        update_moved_vertex();

        // Setup contextual menu
        _menu = new QMenu(_gl_widget);
        {
            QMenu* sub_menu_1 = _menu->addMenu("convert bone to");
            sub_menu_1->addAction("Hermite RBF");
            _map_menutext_enum["Hermite RBF"] = EBone::HRBF;
            sub_menu_1->addAction("Implicit cylinder");
            _map_menutext_enum["Implicit cylinder"] = EBone::CYLINDER;
            sub_menu_1->addAction("SSD");
            _map_menutext_enum["SSD"] = EBone::SSD;
            sub_menu_1->addAction("Precomputed");
            _map_menutext_enum["Precomputed"] = EBone::PRECOMPUTED;

            QMenu* sub_menu_2 = _menu->addMenu("Blending type to");
            sub_menu_2->addAction("Arc gradient controlled");
            _map_menutext_enum["Arc gradient controlled"] = EJoint::GC_ARC_CIRCLE_TWEAK;
            sub_menu_2->addAction("Union with Max");
            _map_menutext_enum["Union with Max"] = EJoint::MAX;
            sub_menu_2->addAction("Bulge");
            _map_menutext_enum["Bulge"] = EJoint::BULGE;
        }
    }

    // -------------------------------------------------------------------------

    virtual ~IO_skeleton(){
    }

    // -------------------------------------------------------------------------

    virtual void mousePressEvent( QMouseEvent* event )
    {
        IO_interface_skin::mousePressEvent(event);
        using namespace Cuda_ctrl;

        //const int x = event->x();
        //const int y = event->y();

        update_gl_matrix();

        if(event->button() == Qt::LeftButton)
        {
            if(_potential_plane._setup)
                _potential_plane._setup = false;
            else
            {
                {
                    const bool rest = _gl_widget->rest_pose();
                    if( _is_ctrl_pushed )
                        _skeleton.select_joint(*_cam, _old_x, _old_y, rest);
                    else if( _is_maj_pushed )
                        _skeleton.unselect(*_cam, _old_x, _old_y, rest);
                    else
                        _skeleton.select_safely(*_cam, _old_x, _old_y, rest);
                }

                update_moved_vertex();

                if(_move_joint_mode)
                {
                    Vec3_cu v = _skeleton.joint_pos(_last_vertex);
                    GLdouble vx, vy, vz;
                    gluProject(v.x, v.y, v.z,
                               _modelview, _projection, _viewport,
                               &vx, &vy, &vz);
                    _mouse_z = vz;
                }
            }
        }

    }

    // -------------------------------------------------------------------------

    virtual void mouseReleaseEvent( QMouseEvent* event )
    {
        IO_interface_skin::mouseReleaseEvent(event);

        using namespace Cuda_ctrl;
        if( _move_joint_mode )
        {
            // update selected vertex
            Cuda_ctrl::_anim_mesh->update_caps(_last_vertex , true, true);
            //update caps every sons
            const std::vector<int>& sons = _skeleton.get_sons( _last_vertex );
            for(unsigned i = 0; i < sons.size(); i++)
                Cuda_ctrl::_anim_mesh->update_caps(sons[i], true, true);

            Cuda_ctrl::_anim_mesh->update_base_potential();
            //Cuda_ctrl::_anim_mesh->update_clusters();
        }
    }

    // -------------------------------------------------------------------------

    virtual void mouseMoveEvent( QMouseEvent* event )
    {
        IO_interface_skin::mouseMoveEvent(event);
        using namespace Cuda_ctrl;

        const int x = event->x();
        const int y = event->y();

        update_gl_matrix();

        if(_is_mid_pushed && _joint > -1)
        {
            //...
        }

        _old_x = x;
        _old_y = _cam->height() - y;
    }

    // -------------------------------------------------------------------------

    virtual void wheelEvent( QWheelEvent* event )
    {
        using namespace Cuda_ctrl;
        const std::vector<int>& set = _skeleton.get_selection_set();
        float numDegrees = event->delta() / 8.f;
        float numSteps   = numDegrees / 15.f;

        if(_potential_plane._setup)
        {
            if(event->buttons() == Qt::NoButton )
            {
                Vec3_cu org  = _potential_plane._org;
                Vec3_cu n    = _potential_plane._normal;
                _potential_plane._org = org + n * numSteps;
            }
        }
        else if(_is_maj_pushed && set.size() > 0)
        {
            int bone_id = set[set.size()-1];
            _anim_mesh->incr_junction_rad(bone_id, numSteps/16.f);
            _display._raytrace_again = true;
        }
        else
            IO_interface_skin::wheelEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void keyPressEvent(QKeyEvent* event)
    {
        using namespace Cuda_ctrl;

        if(event == QKeySequence::SelectAll)
            Cuda_ctrl::_skeleton.select_all();

        IO_interface_skin::keyPressEvent(event);

        QString t = event->text();
        QChar c = t[0];

        switch (c.toLatin1())
        {
        case 'j':
        {
            if(!_move_joint_mode && !event->isAutoRepeat())
            {
                _move_joint_mode = true;
                _gl_widget->set_rest_pose( true );
                push_msge("Hold the key to move the joint and left click");
            }
        }break;

        }//END_SWITCH

        if(_is_space_pushed)
            trigger_menu();
    }

    // -------------------------------------------------------------------------

    void trigger_menu()
    {
        using namespace Cuda_ctrl;

        // Print and wait for the contextual menu
        QAction* selected_item = _menu->exec(QCursor::pos());
        // Key space release event is unfortunately eaten by the
        // QMenu widget we have to set it back by hand...
        _is_space_pushed = false;

        if(selected_item == 0) return;

        QMenu* menu = (QMenu*)selected_item->parentWidget();
        const std::vector<int>& set = _skeleton.get_selection_set();
        if(set.size() <= 0) {
            push_msge("Error : please select a least one bone to convert");
            return;
        }

        std::string item_text = selected_item->text().toStdString();

        // TODO: check if the entry doesn't exists

        bool type_changed = false;
        for(unsigned i = 0; i < set.size(); i++)
        {
            if( menu->title().compare("convert bone to") == 0  )
            {
                int type = _map_menutext_enum[item_text];
                int bone_type = _skeleton.get_bone_type(set[i]);

                if(bone_type == EBone::PRECOMPUTED &&
                        type == EBone::PRECOMPUTED)
                {
                    std::cout << "Error : bone (" << i  << ") is a precomputed bone, it can't be precomputed" << std::endl;
                }
                else if(bone_type == EBone::SSD &&
                        type      == EBone::PRECOMPUTED)
                {
                    std::cout << "Error : bone (" << i << ") is an SSD bone, it can't be precomputed" << std::endl;
                }
                else
                {
                    _anim_mesh->set_bone_type(set[i], type);
                    type_changed = true;
                }
            }
            else if( menu->title().compare("Blending type to") == 0  )
            {
                EJoint::Joint_t type = (EJoint::Joint_t)_map_menutext_enum[item_text];
                _skeleton.set_joint_blending(set[i], type);
                type_changed = true;
            }
            Cuda_ctrl::_display._raytrace_again = true;
        }

        if(type_changed) _anim_mesh->update_base_potential();
    }

    // -------------------------------------------------------------------------

    virtual void keyReleaseEvent(QKeyEvent* event)
    {
        IO_interface_skin::keyReleaseEvent(event);

        QString t = event->text();
        QChar c = t[0];

        switch (c.toLatin1())
        {
        case 'j':
        {
            if( _move_joint_mode && !event->isAutoRepeat())
            {
                _move_joint_mode = false;
                _gl_widget->set_rest_pose( false );
            }
        }break;

        }//END_SWITCH
    }

    // -------------------------------------------------------------------------

    virtual void update_frame_gizmo()
    {
    }

    // -------------------------------------------------------------------------

    bool _janim_on;

private:

    // -------------------------------------------------------------------------
    /// @name Tools
    // -------------------------------------------------------------------------
    void update_moved_vertex()
    {
        using namespace Cuda_ctrl;
        const std::vector<int>& set = _skeleton.get_selection_set();

        _joint = -1;
        _last_vertex  = -1;
        if(set.size() > 0)
        {
            int idx = set[set.size()-1];
            _last_vertex  = idx;
            _joint = idx;

            _curr_joint_lcl = kinec()->get_user_lcl_parent( _joint );
            _curr_joint_org = g_skel->joint_anim_frame(_joint).get_translation();
        }

        //update_frame_gizmo();

        _nb_selected_vert = set.size();

        IBL::Ctrl_setup shape;
        if(_joint != -1)
            shape = _skeleton.get_joint_controller(_joint);
        else
            shape = _operators.get_global_controller();

        _main_win->update_ctrl_spin_boxes(shape);
    }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    float _mouse_z;          ///< z depth when "_moved_node" has been clicked

    int   _joint;            ///< last joint id being selected
    // TODO: checke this attribute it might be redundant with _joint
    int   _last_vertex;      ///< joint being selected

    int   _nb_selected_vert; ///< current number of skeleton's joints selected
    bool  _move_joint_mode;  ///< enable user to move the skeleton joints in rest pose

    /// joint local transformation (according to the parent)
    /// when clicking on the gizmo
    Transfo _curr_joint_lcl;

    /// joint global position when clicking on the gizmo
    Vec3_cu _curr_joint_org;

    float _selection_hysteresis;

    QMenu* _menu;            ///< a contextual menu triggered with space key

    /// Map of the textual entries of menu to the enum field in Bone_type
    /// @see Bone_type
    std::map<std::string, int> _map_menutext_enum;
};

#endif // IO_SKELETON_HPP__
