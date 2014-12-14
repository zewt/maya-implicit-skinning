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
#ifndef MSGE_STACK_HPP_
#define MSGE_STACK_HPP_

#include <deque>
#include <QString>
#include <QTimer>
#include <QTime>
#include <QFont>
#include <QtOpenGL/QGLWidget>

#include "text_renderer.hpp"

/** @class Msge_stack
    @brief Utility to print messages a limited amount of time on a viewport

    This class is an utility to prompt messages over the viewport.
    Messages are stacked and poped at regular intervals
*/

#define MSGE_LIFE_TIME 3000 // in ms
#define FONT_SIZE 17

class Msge_stack : QObject{
    Q_OBJECT
public:

    Msge_stack(const QGLWidget *gl_widget) : QObject() {
        QFont font;
        font.setStyleHint( QFont::Courier, QFont::PreferAntialias );
        font.setPixelSize(FONT_SIZE);
        _text_renderer.setup( gl_widget, font );
        QObject::connect(&_timer, SIGNAL(timeout()), this, SLOT(time_out()));
    }

    /// Draw the message stack at (x,y) position. (bottom left is (0,0))
    void draw(int x, int y){
        _text_renderer.begin();
        for (unsigned i = 0; i < _msge.size(); ++i)
            _text_renderer.print( x, y-i*(FONT_SIZE+1), _msge[i] );
        _text_renderer.end();
    }

    /// Push a message 'msge' to the stack it will be poped after a delay equal
    /// to MSGE_LIFE_TIME.
    /// @param add_date If true concatenate the current time before the string
    /// 'msge'
    void push(const QString& msge, bool add_date = false){
        _timer.start(MSGE_LIFE_TIME);
        QString date = "";
        if(add_date)
            date = "["+QTime::currentTime().toString("hh:mm:ss")+"] ";
        _msge.push_back(date+msge);
    }

    /// @return the timer used to pop messages progressively in the stack.
    /// Each time The timer is triggered a message is erased from the bottom
    /// of the stack. One can connect the timer to another method with this
    /// getter
    const QTimer& get_timer() const { return _timer; }

private slots:
    void time_out(){
        if(_msge.size() == 0) _timer.stop();
        else                  _msge.pop_front();
    }
private:
    /// Render text on opengl window
    TextRenderer _text_renderer;
    /// Stack of messages progressively erased
    std::deque<QString> _msge;
    /// The timer pops messages at constant intervalles
    QTimer _timer;
};

#endif // MSGE_STACK_HPP_
