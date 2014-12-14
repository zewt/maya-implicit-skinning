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
#ifndef IO_MESH_EDIT_HPP__
#define IO_MESH_EDIT_HPP__

#include "maths/vec2i_cu.hpp"
#include "SKIN/IO_skeleton.hpp"

/**
 * @name IO_mesh_edit
 * @brief Handle mouse and keys for mesh editing (vertex selection/painting)
*/
class IO_mesh_edit : public IO_skeleton {
public:
    IO_mesh_edit(OGL_widget_skin* gl_widget) :
        IO_skeleton(gl_widget),
        _is_edit_on(false)
    {  }

    // -------------------------------------------------------------------------

    virtual ~IO_mesh_edit(){
        Cuda_ctrl::_anim_mesh->reset_selection();
    }

    // -------------------------------------------------------------------------

    virtual void mousePressEvent( QMouseEvent* event ){
        IO_skeleton::mousePressEvent(event);
        using namespace Cuda_ctrl;

        const int x = event->x();
        const int y = event->y();
        _old_mouse = Vec2i_cu(x, y);

        if(_is_left_pushed && _is_edit_on)
        {
            if(_is_ctrl_pushed)
                // Add to previous selection
                select(x, y);
            else if(_is_maj_pushed)
                // Remove from previous
                unselect(x, y);
            else{
                // Discard old selection and add the new one
                _anim_mesh->reset_selection();
                select(x, y);
            }
        }

        if( _is_left_pushed && is_paint_on() )
            paint(x, y);
        //event->ignore();
    }

    // -------------------------------------------------------------------------

    virtual void mouseReleaseEvent( QMouseEvent* event ){
        IO_skeleton::mouseReleaseEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void mouseMoveEvent( QMouseEvent* event ){
        IO_skeleton::mouseMoveEvent(event);

        const int x = event->x();
        const int y = event->y();
        Vec2i_cu m(x, y);

        if( _is_left_pushed && is_paint_on() && (_old_mouse - m).norm() > 0.5f )
            paint(x, y);

        // TODO: update painted attributes when mouse release

        _old_mouse = m;
    }

    // -------------------------------------------------------------------------

    virtual void wheelEvent( QWheelEvent* event ){
        using namespace Cuda_ctrl;
        if(_gl_widget->get_heuristic()->_type == Selection::CIRCLE &&
           _is_ctrl_pushed)
        {
            float numDegrees = event->delta() / 8.f;
            float numSteps   = numDegrees / 15.f;

            Selection_circle<int>* h = (Selection_circle<int>*)_gl_widget->get_heuristic();
            float tmp = h->_rad + 10*numSteps;
            h->_rad   = tmp < 0.f ? 0.f : tmp;
        }else
            IO_skeleton::wheelEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void keyPressEvent(QKeyEvent* event){
        IO_skeleton::keyPressEvent(event);
        if(event->key() == Qt::Key_Tab){
            _is_edit_on = !_is_edit_on;
            Cuda_ctrl::_anim_mesh->set_display_points(_is_edit_on);
        }
    }

    // -------------------------------------------------------------------------

    virtual void keyReleaseEvent(QKeyEvent* event){
        IO_skeleton::keyReleaseEvent(event);
    }

    // -------------------------------------------------------------------------

protected:

    bool is_paint_on(){ return _main_win->toolBar_painting->is_paint_on(); }


    // -------------------------------------------------------------------------

    void paint(int x, int y)
    {
        Widget_painting*  paint_wgt  = _main_win->toolBar_painting->_paint_widget;
        Widget_selection* select_wgt = _main_win->toolBar->_wgt_select;
        Selection_circle<int>* h = (Selection_circle<int>*)_gl_widget->get_heuristic();
        Animesh::Paint_setup setup;
        setup._brush_radius = h->_rad;
        setup._rest_pose = _gl_widget->rest_pose();
        setup._val = paint_wgt->dSpinB_strength->value();
        setup._backface_cull = !select_wgt->toolB_backface_select->isChecked();
        setup._x = x;
        setup._y = y;
        g_animesh->paint(EAnimesh::PT_SSD_INTERPOLATION, setup, *_cam);
    }

    // -------------------------------------------------------------------------

    void select(float x, float y)
    {
        Cuda_ctrl::_anim_mesh->select(*_cam,
                                      x, y,
                                      _gl_widget->get_heuristic(),
                                      _gl_widget->rest_pose());
    }

    // -------------------------------------------------------------------------

    void unselect(float x, float y)
    {
        Cuda_ctrl::_anim_mesh->unselect(*_cam,
                                        x, y,
                                        _gl_widget->get_heuristic(),
                                        _gl_widget->rest_pose());

    }

    // -------------------------------------------------------------------------


    /*-----------*
    | Attributes |
    *-----------*/
    bool     _is_edit_on;
    Vec2i_cu _old_mouse;  ///< mouse position at last click
};

#endif // IO_MESH_EDIT_HPP__
