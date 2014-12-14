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
#if 0

#ifndef IO_SKELETON_HPP__
#define IO_SKELETON_HPP__

#include "IO_interface.hpp"
#include "objects/object.hpp"

#include <vector>

/** @brief
*/

class IO_selection : public IO_interface {
public:

    IO_selection(OGL_widget* gl_widget) :
        IO_interface( gl_widget )
    {

    }

    // -------------------------------------------------------------------------

    virtual ~IO_selection()
    {

    }

    // -------------------------------------------------------------------------

    virtual void mousePressEvent( QMouseEvent* event )
    {
        IO_interface::mousePressEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void mouseReleaseEvent( QMouseEvent* event )
    {
        IO_interface::mouseReleaseEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void mouseMoveEvent( QMouseEvent* event )
    {
        IO_interface::mouseMoveEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void wheelEvent( QWheelEvent* event )
    {
        IO_interface::wheelEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void keyPressEvent(QKeyEvent* event)
    {
        IO_interface::keyPressEvent(event);
    }

    // -------------------------------------------------------------------------

    virtual void keyReleaseEvent(QKeyEvent* event)
    {
        IO_interface::keyReleaseEvent(event);
    }

    // -------------------------------------------------------------------------

private:

    /*------*
    | Tools |
    *------*/

    /*-----------*
    | Attributes |
    *-----------*/
    std::vector<Object*> _objects; ///< objects to be selected
};

#endif // IO_SKELETON_HPP__

#endif
