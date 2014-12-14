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
#ifndef IO_RBF_HPP__
#define IO_RBF_HPP__

#include "SKIN/IO_mesh_edit.hpp"
#include <cmath>

/// The current mesh
#include "mesh.hpp"
#include "conversions.hpp"
extern Mesh* g_mesh;

/** @brief Handle mouse and keys for RBF sample (move, delete)

  @see IO_mesh_edit
*/
class IO_RBF : public IO_mesh_edit {
public:
    IO_RBF(OGL_widget_skin* gl_widget) :
        IO_mesh_edit(gl_widget)
    {
    }

    virtual ~IO_RBF(){
        _gl_widget->gizmo()->reset_constraint();
        Cuda_ctrl::_anim_mesh->reset_samples_selection();
    }

    // -------------------------------------------------------------------------

    virtual void mousePressEvent( QMouseEvent* event ){
        IO_mesh_edit::mousePressEvent(event);
        using namespace Cuda_ctrl;

        const int x = event->x();
        const int y = event->y();
        _pix_clicked = Vec2_cu((float)x, (float)y);

        if(event->button() == Qt::LeftButton )
        {
            // No axis has been choose we test for normal selection
            if( !_is_gizmo_grabed )
            {
                if(_is_ctrl_pushed)
                    // Add to previous selection
                    select_samples(x, y);
                else if(_is_maj_pushed)
                    // Remove from previous
                    unselect_samples(x, y);
                else{
                    // Discard old selection and add the new one
                    _anim_mesh->reset_samples_selection();
                    select_samples(x, y);
                }

                if(are_samples_selected()){
                    set_frame_pos();
                    set_frame_axis();
                    _old_tr = gizmo()->frame();
                }
            }
        }

    }

    // -------------------------------------------------------------------------

    virtual void mouseReleaseEvent( QMouseEvent* event ){
        IO_mesh_edit::mouseReleaseEvent(event);
        using namespace Cuda_ctrl;
        //const int x = event->x();
        //const int y = event->y();

        if( !_is_left_pushed && are_samples_selected() )
        {
            Transfo g = gizmo_global_transfo();
            _anim_mesh->transform_selected_samples(  g );
            Cuda_ctrl::_anim_mesh->update_base_potential();
            Cuda_ctrl::_display._raytrace_again = true;

            set_frame_pos();
            set_frame_axis();
            _old_tr = gizmo()->frame();
            _gizmo_tr = TRS();
        }

    }

    // -------------------------------------------------------------------------

    virtual void mouseMoveEvent( QMouseEvent* event )
    {
        _janim_on = !are_samples_selected();
        IO_mesh_edit::mouseMoveEvent(event);
        _janim_on = true;
/*
        using namespace Cuda_ctrl;

        const int x = event->x();
        const int y = event->y();
        Vec2_cu p((float)x, (float)y);

        if(_is_left_pushed && (_pix_clicked - p).norm() > 1.5f ){

        }
*/
    }

    // -------------------------------------------------------------------------

    virtual void keyPressEvent(QKeyEvent* event){
        IO_mesh_edit::keyPressEvent(event);
        using namespace Cuda_ctrl;

        if(event->key() == Qt::Key_Delete){
            _anim_mesh->delete_selected_samples();
            Cuda_ctrl::_anim_mesh->update_base_potential();
            Cuda_ctrl::_display._raytrace_again = true;
        }
        else if( event->key() == Qt::Key_Return ||  event->key() == Qt::Key_Enter )
        {
            if( _is_edit_on)
                gen_hrbf_samples_from_mesh_selection();
        }
    }

    // -------------------------------------------------------------------------

    virtual void keyReleaseEvent(QKeyEvent* event)
    {
        IO_mesh_edit::keyReleaseEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void wheelEvent( QWheelEvent* event )
    {
        const std::vector<int>& set = Cuda_ctrl::_skeleton.get_selection_set();

        if(_is_maj_pushed && set.size() > 0)
        {
            float numDegrees = event->delta() / 8.f;
            float numSteps   = numDegrees / 15.f;

            int bone_id = set[set.size()-1];
            Cuda_ctrl::_anim_mesh->incr_junction_rad(bone_id, numSteps/16.f);
            Cuda_ctrl::_anim_mesh->update_base_potential();
            Cuda_ctrl::_display._raytrace_again = true;
        }else
            IO_mesh_edit::wheelEvent(event);
    }

    // -------------------------------------------------------------------------


    void update_frame_gizmo()
    {
        if( !are_samples_selected() )
            IO_skeleton::update_frame_gizmo();
        else
        {
            Transfo global_tr = gizmo_global_transfo();
            gizmo()->set_transfo( global_tr * _old_tr );
        }
    }


private:
    // -------------------------------------------------------------------------
    /// @name Tools
    //  ------------------------------------------------------------------------

    Transfo gizmo_global_transfo()
    {
        TRS tr = _gizmo_tr;
        Vec3_cu v   = gizmo()->old_frame() * tr._translation;
        Vec3_cu a   = gizmo()->old_frame() * tr._axis;
        Vec3_cu org = _old_tr.get_org();

        return Transfo::translate( v ) * Transfo::rotate(org ,a, tr._angle);
    }

    // -------------------------------------------------------------------------

    Select_type<Samp_id>* get_heuristic(int type)
    {
        switch(type){
        case Selection::NEAREST:
            return new Selection_nearest<Samp_id>();
        case Selection::CIRCLE:
            return new Selection_circle<Samp_id>();
        default:
            return 0;
        }
    }

    // -------------------------------------------------------------------------

    void gen_hrbf_samples_from_mesh_selection()
    {
        const std::vector<int>& set_skel = Cuda_ctrl::_skeleton.get_selection_set();
        if(set_skel.size() == 0) return;

        const std::vector<int>& set = Cuda_ctrl::_anim_mesh->get_selected_points();

        std::vector<Vec3_cu> p(set.size());
        std::vector<Vec3_cu> n(set.size());
        for(unsigned i = 0; i < set.size(); i++)
        {
            const int id = set[i];

            p[i] = g_mesh->get_vertex( id );
            n[i] = g_mesh->get_normal( id );
        }

        Cuda_ctrl::_anim_mesh->add_samples(set_skel[set_skel.size()-1], p, n);
        Cuda_ctrl::_anim_mesh->update_base_potential();
    }

    // -------------------------------------------------------------------------

    Vec3_cu get_sample(const Samp_id& id)
    {
        using namespace Cuda_ctrl;
        return _gl_widget->rest_pose() ?
                    _anim_mesh->get_sample_pos(id) :
                    _anim_mesh->get_sample_anim_pos(id);
    }

    // -------------------------------------------------------------------------

    Vec3_cu get_normal(const Samp_id& id)
    {
        using namespace Cuda_ctrl;
        return _gl_widget->rest_pose() ?
                    _anim_mesh->get_sample_normal(id) :
                    _anim_mesh->get_sample_anim_normal(id);
    }

    // -------------------------------------------------------------------------

    bool are_samples_selected()
    {
        using namespace Cuda_ctrl;
        const std::vector<Samp_id>& list = _anim_mesh->get_selected_samples();
        return list.size() != 0;
    }

    // -------------------------------------------------------------------------

    void set_frame_pos()
    {
        using namespace Cuda_ctrl;
        const std::vector<Samp_id>& list = _anim_mesh->get_selected_samples();

        if( list.size() == 0) return;

        Vec3_cu cog = Vec3_cu(0.f, 0.f, 0.f);
        for(unsigned i = 0; i < list.size(); i++)
            cog = cog + get_sample( list[i] );

        cog = cog * (1.f/(float)list.size());

        gizmo()->set_org( cog );
    }

    // -------------------------------------------------------------------------

    void set_frame_axis()
    {
        using namespace Cuda_ctrl;
        Vec3_cu fx = Vec3_cu::unit_x();
        Vec3_cu fy = Vec3_cu::unit_y();
        Vec3_cu fz = Vec3_cu::unit_z() * -1;

        if( !are_samples_selected() ) return;

        if( _main_win->local_frame->isChecked() )
        {
            const std::vector<int>& set = _skeleton.get_selection_set();
            if(set.size() > 0)
            {
                int id = set[set.size()-1];
                _skeleton.joint_anim_frame(id, fx, fy, fz);
            }
        }
        else if( _main_win->checkB_align_with_normal->isChecked() )
        {
            const std::vector<Samp_id>& set = _anim_mesh->get_selected_samples();
            if( set.size() > 0 )
            {
                fx = Vec3_cu::zero();
                for( unsigned i = 0; i < set.size(); i++)
                    fx = fx + get_normal( set[i] );
                fx.coordinate_system(fy, fz);
            }
        }

        gizmo()->set_frame( Mat3_cu(fx, fy, fz) );
    }

    // -------------------------------------------------------------------------

    bool select_samples(float x, float y)
    {
        int type = _gl_widget->get_heuristic()->_type;
        Select_type<Samp_id>* heur = get_heuristic(type);

        bool s = Cuda_ctrl::_anim_mesh->select_samples(*_cam,
                                                       x, y,
                                                       heur,
                                                       _gl_widget->rest_pose());

        delete heur;
        return s;
    }

    // -------------------------------------------------------------------------

    bool unselect_samples(float x, float y)
    {
        int type = _gl_widget->get_heuristic()->_type;
        Select_type<Samp_id>* heur = get_heuristic(type);

        bool s = Cuda_ctrl::_anim_mesh->unselect_samples(*_cam,
                                                         x, y,
                                                         heur,
                                                         _gl_widget->rest_pose());
        delete heur;
        return s;
    }

    // -------------------------------------------------------------------------

    Vec2_cu _pix_clicked;  ///< pixel position when mouse click
    Transfo _old_tr;       ///<
};

#endif // IO_RBF_HPP__
