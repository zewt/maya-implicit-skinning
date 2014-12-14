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
#ifndef CLICKABLE_FRAME_HPP__
#define CLICKABLE_FRAME_HPP__

#include <QFrame>
#include <QMouseEvent>

/** @class Clickable_frame
    @brief A QFrame which throws a signal when left clicked on.
*/

class Clickable_frame : public QFrame{
    Q_OBJECT
public:

    Clickable_frame(QWidget* parent = 0) : QFrame(parent) {}

    void mousePressEvent( QMouseEvent* event ){
        if(event->button() == Qt::LeftButton)
        {
            emit leftclick_on();
        }
    }

    /// Set the background of the frame
    /// @warning The method erased the previous QFrame style sheet
    void set_background(const QColor& cl){
        setStyleSheet("background-color: rgb("+
                      QString::number(cl.red())+", "+
                      QString::number(cl.green())+", "+
                      QString::number(cl.blue())+");");
    }

signals:
    void leftclick_on();
};

#endif // CLICKABLE_FRAME_HPP__
