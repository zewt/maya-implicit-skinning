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
#ifndef IO_GRAPH_HPP__
#define IO_GRAPH_HPP__

#include "SKIN/IO_interface_skin.hpp"

/** @brief Handle mouse and keys for graph edition

  @see IO_interface
*/
class IO_graph : public IO_interface_skin {
public:
    IO_graph(OGL_widget_skin* gl_widget) :
        IO_interface_skin(gl_widget),
        _moved_node(-1),
        _mouse_z(0.f)
    {
        _cursor = Vec3_cu(0.f, 0.f, 0.f);
    }

    // -------------------------------------------------------------------------

    virtual ~IO_graph(){ }

    // -------------------------------------------------------------------------

    virtual void mousePressEvent( QMouseEvent* event ){
        IO_interface_skin::mousePressEvent(event);
        using namespace Cuda_ctrl;

        update_gl_matrix();

        if(event->button() == Qt::LeftButton)
        {
            if( _graph.select_node(_old_x, _old_y) )
            {
                _moved_node = _graph.get_selected_node();
                //std::cout << "selected node id: " << _moved_node << std::endl;
                Vec3_cu v = _graph.get_vertex(_moved_node);
                GLdouble vx, vy, vz;
                gluProject(v.x, v.y, v.z,
                           _modelview, _projection, _viewport,
                           &vx, &vy, &vz);
                _mouse_z = vz;
            }
            else
            {
                float cx = _old_x;
                float cy = _old_y;
                float cz;
                int previous_node = _moved_node;
                glReadPixels( _old_x, _old_y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &cz);

                GLdouble ccx,ccy,ccz;
                gluUnProject(cx, cy, cz,
                             _modelview, _projection, _viewport,
                             &ccx, &ccy, &ccz);

                _cursor.set(ccx, ccy, ccz);
                _moved_node = _graph.push_vertex( _cursor );
                _graph.set_selected_node( _moved_node );

                gluProject(_cursor.x, _cursor.y, _cursor.z,
                           _modelview, _projection, _viewport,
                           &ccx, &ccy, &ccz);

                _mouse_z = ccz;

                //std::cout << "push vertex ";
                //std::cout << " (" << _old_x << "," << _old_y << ")\n" << std::endl;

                if(_is_ctrl_pushed && previous_node > -1)
                    _graph.push_edge(_moved_node, previous_node);
            }
        }

        if(event->button() == Qt::MidButton)
        {
            if( _graph.select_node(_old_x, _old_y) ){
                if(_moved_node > -1){
                    _graph.push_edge(_graph.get_selected_node(), _moved_node);
                    _graph.set_selected_node(_moved_node);
                }
            }
        }
    }

    // -------------------------------------------------------------------------

    virtual void mouseReleaseEvent( QMouseEvent* event ){
        IO_interface_skin::mouseReleaseEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void mouseMoveEvent( QMouseEvent* event ){
        IO_interface_skin::mouseMoveEvent(event);
        using namespace Cuda_ctrl;
        const int x = event->x();
        const int y = event->y();

        update_gl_matrix();

        if(_is_left_pushed){
            GLdouble ccx,ccy,ccz;
            float cx = x, cy = _cam->height()-y, cz = _mouse_z;
            gluUnProject(cx, cy, cz,
                         _modelview, _projection, _viewport,
                         &ccx, &ccy, &ccz);
            _graph.set_vertex(_moved_node, Vec3_cu(ccx, ccy, ccz));
        }
    }

    // -------------------------------------------------------------------------

    virtual void keyPressEvent(QKeyEvent* event){
        IO_interface_skin::keyPressEvent(event);

        // Check special keys
        switch( event->key() ){
        case Qt::Key_Delete:
            if(_moved_node > -1){
                Cuda_ctrl::_graph.remove(_moved_node);
            }
            _moved_node = -1;
            break;
        }
    }

    // -------------------------------------------------------------------------

    virtual void wheelEvent( QWheelEvent* event ){
        IO_interface_skin::wheelEvent(event);
    }

    // -------------------------------------------------------------------------

private:
    int _moved_node;    ///< current graph node to be moved

    float _mouse_z;     ///<
    Vec3_cu _cursor;  ///<


};

#endif // IO_GRAPH_HPP__
