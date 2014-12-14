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
#ifndef TEXT_RENDERER_HPP_
#define TEXT_RENDERER_HPP_

/***************************************************************************
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   copyright            : (C) 2006 by Benoit Jacob                       *
 ***************************************************************************/

#include <QtOpenGL/QGLWidget>
#include <QPainter>
#include <QImage>
#include <QChar>
#include <QHash>

/** This is a helper class for TextRenderer, and should probably never be
* used directly. See TextRenderer.
*
* The CharRenderer class represents a character stored as OpenGL rendering
* data : a texture object and a display list mapping it on a quad and then
* translating to the right of it.
*
* See the m_charTable member of TextRenderer for an example of use of
* this class.
*/
class CharRenderer
{
    protected:
        /**
         * The OpenGL texture object
         */
        GLuint m_texture;

        /**
         * The OpenGL display list
         */
        GLuint m_displayList;

        /**
         * Width and height in pixels of the rendered character
         */
        int m_width, m_height;

    public:
        CharRenderer();
        ~CharRenderer();
        bool initialize( QChar c, const QFont &font );
        inline void draw()
        {
            glCallList( m_displayList );
        }
};


/** This class renders text inside a QGLWidget. It replaces the functionality
* of QGLWidget::renderText(). The advantages over renderText() include:
*  - supports any font, any character encoding supported by Qt
*    (renderText is 8-bit-only and can only use "OpenGL-compatible" fonts)
*  - does not use any library outside Qt (renderText uses FreeType on X11)
*  - renders characters as textured quads instead of calling glDrawPixels,
*    which does not make much of a difference on MesaGL, but can be a lot
*    faster and safer with other (buggy) OpenGL implementations. It will also
*    allow to add more graphical effects in the future, like rotation,
*    if we ever need that.
*  - the characters are stored as 8bpp Alpha, which takes 4 times less
*    memory than the 32bpp RGBA used by renderText.
*  - the characters are rendered on-the-fly on the first time they appear
*    in a QString being printed. This is achieved using a QHash to test whether
*    a character has already been rendered.
*
* Recommended usage:
* The TextRender class is meant to be used from inside a child class of
* QGLWidget, say MyGLWidget.
*
* In the declaration of MyGLWidget, please declare a TextRenderer member:
*
* @code
class MyGLWidget : public QGLWidget
{
    ...
    TextRenderer m_textRenderer;
    ...
};
* @endcode
*
* Now, in the constructor of MyGLWidget, please call setup() along these lines:
*
* @code
    QFont f;
    f.setStyleHint( QFont::SansSerif, QFont::PreferAntialias );
    m_textRenderer.setup( this, f );
* @endcode
*
* The setup() method should be called only once, which means you have to choose
* a font once and for all, in the lifetime of your TextRenderer. Any QFont can
* be used, the above is just an example. Now, to actually render text, in
* the MyGLWidget::paintGL() method, you can call

* @code
    m_textRenderer.print( x, y, string );
* @endcode

* where x,y are ints and string is any QString. If you want to choose a color,
* please call glColor3f or glColor4f before calling print(). Of course you can
* also call qglColor. You can achieve semitransparent text at
* no additional cost by choosing a semitransparent color.
*
* If you wish to do several calls to print(), it will improve performance
* to enclose them between a call to begin() and a call to end(), like that:
*
* @code
    m_textRenderer.begin();
    m_textRenderer.print( x1, y1, string1 );
    m_textRenderer.print( x2, y2, string2 );
    m_textRenderer.print( x3, y2, string3 );
    m_textRenderer.end();
* @endcode
*
* Please make sure, though, that no relevant OpenGL state change occurs between
* begin() and end(), except the state changes performed by the TextRenderer
* itself. In other words, please avoid calling glSomething() between begin() and
* end(), except if you are sure that this call won't perform a relevant state
* change.
*
* The print() method when called alone, or the begin()-print()-end() group,
* do restore the OpenGL state as they found it, including the matrices.
*
* If you experience rendering problems, you can try the following:
* - disable some OpenGL state bits. For instance, TextRenderer automatically
*   disables fog and lighting during rendering, because it doesn't work
*   correctly with them enabled. There probably are other OpenGL state bits
*   that have to be disabled, so if your program enables some of them, you
*   might have to disable them before rendering text.
* - if you experience poor font quality, please consider using an antialiased
*   font.
*
* @author Benoit Jacob
*/
class TextRenderer
{
    protected:
        /**
         * The font used for rendering the chars. This is set
         * once and for all by setup(). Note that it is stored
         * by value, so the caller doesn't have to keep it alive.
         */
        QFont m_font;

        /**
         * This hash gives the correspondence table between QChars
         * (the keys) and the corresponding CharRenderers (the values).
         * Every time a QChar is being met, either it is found in this
         * table, in which case it can be directly rendered, or it is
         * not found, in which case a new CharRenderer is created for
         * it and added to this table.
         */
        QHash<QChar, CharRenderer*> m_charTable;

        /**
         * The QGLWidget in which to render. This is set
         * once and for all by setup().
         */
        const QGLWidget *m_glwidget;

        /**
         * This equals true if begin() has been called, but end() hasn't
         * since.
         */
        GLboolean m_isBetweenBeginAndEnd;

        /**
         * These members are used to remember the OpenGL state in order
         * to be able to restore it after rendering. See do_end().
         */
        GLboolean m_wasEnabled_LIGHTING;
        GLboolean m_wasEnabled_TEXTURE_2D;
        GLboolean m_wasEnabled_FOG;
        GLboolean m_wasEnabled_BLEND;
        GLboolean m_wasEnabled_DEPTH_TEST;

        /**
         * Stores the relevant part of the OpenGL state, and prepares
         * for rendering
         */
        void do_begin();

        /**
         * Restores the OpenGL state
         */
        void do_end();

    public:
        TextRenderer();
        ~TextRenderer();

        /**
         * This should be called only once, before any printing occurs.
         * @param glwidget The QGLWidget in which to render.
         * See m_glwidget member.
         * @param font The QFont to use. See m_font member.
         */
        void setup( const QGLWidget *glwidget, const QFont &font );

        /**
         * Prints text at the position (x,y) in window coordinates
         * (0,0) is the bottom left corner
         * @param x the x-coordinate
         * @param y the y-coordinate
         * @param string the QString to print
         */
        void print( int x, int y, const QString &string);

        /**
         * Call this before doing multiple calls to print(). This is
         * not necessary, but will improve performance. Don't forget,
         * then, to call end() after.
         */
        void begin();

        /**
         * Call this after having called begin() and print().
         */
        void end();
};

#endif // TEXT_RENDERER_HPP_
