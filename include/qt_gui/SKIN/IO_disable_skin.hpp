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
#ifndef IO_DISABLE_SKIN_HPP__
#define IO_DISABLE_SKIN_HPP__

#include "SKIN/IO_interface_skin.hpp"

/** @brief Disables mouse and keys
  @see IO_interface
*/

class IO_disable_skin : public IO_interface_skin {
public:
    IO_disable_skin(OGL_widget_skin* gl_widget) : IO_interface_skin(gl_widget){ }

    void mousePressEvent  (QMouseEvent* e){ e->ignore(); }
    void mouseReleaseEvent(QMouseEvent* e){ e->ignore(); }
    void mouseMoveEvent   (QMouseEvent* e){ e->ignore(); }
    void wheelEvent       (QWheelEvent* e){ e->ignore(); }
    void keyPressEvent    (QKeyEvent*   e){ e->ignore(); }
    void keyReleaseEvent  (QKeyEvent*   e){ e->ignore(); }
};

#endif // IO_DISABLE_HPP__
