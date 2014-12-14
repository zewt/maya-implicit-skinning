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
#ifndef GL_SAVE_H
#define GL_SAVE_H

/** @file glsave.hpp
  small util to ease the burden of keeping openGL states
  safe after a step : at destruction, restore a state to the value
  it had at construction time.

  @warning Each class must ba called with a variable name i.e declared like
  this : Class var_name;
  An anonimous construction Class() would save and restore immediatly the state.
*/

#include "port_glew.h"
#include "glassert.h"

// -----------------------------------------------------------------------------

/** @class GLEnabledSave
 *  @brief enable/disable opengl state and restore it at object destruction
*/
class GLEnabledSave {
    GLenum    _capacity;
    GLboolean _value;
    bool      _saved;

public:
    ///  @param gl_state          Opengl state to save
    ///  @param change_gl_state   do we change it while saving ?
    ///  @param change_to         If we do change it,  to what state ?
    ///                           (true:enables|false:disables)
    GLEnabledSave (GLenum gl_state,
                   bool change_gl_state = false,
                   GLboolean change_to = false) :
        _capacity (gl_state)
    {
        save (change_gl_state, change_to);
    }

    GLEnabledSave (bool, GLenum capacity_) :
            _capacity (capacity_)
    {
        _saved = false;
    }

    ~GLEnabledSave () { restore(); }

    void save(bool change = false, GLboolean enabled = false)
    {
        _saved = true;
        glAssert (_value = glIsEnabled(_capacity));

        if((_value != enabled) && change)
        {
            if(enabled) glAssert( glEnable (_capacity) );
            else        glAssert( glDisable(_capacity) );
        }
    }

    void restore ()
    {
        if (!_saved) return;

        if (glIsEnabled (_capacity) != _value)
        {
            if (_value) glAssert( glEnable (_capacity) );
            else         glAssert( glDisable(_capacity) );
        }
    }
};

// -----------------------------------------------------------------------------

/** @class GLBlendSave
    blending, saves GL_BLEND_SRC, GL_BLEND_DST, GL_BLEND_EQUATION_RGB and
    GL_BLEND_EQUATION_ALPHA
*/
class GLBlendSave {
private:
    GLint _src, _dst, _eqRGB, _eqAlpha;
public:
    GLBlendSave (    ) { save();    }
    ~GLBlendSave(    ) { restore(); }
    GLBlendSave (bool) { _src = -1; }

    void save () {
        glAssert (glGetIntegerv (GL_BLEND_SRC, &_src));
        glAssert (glGetIntegerv (GL_BLEND_DST, &_dst));
        glAssert (glGetIntegerv (GL_BLEND_EQUATION_RGB, &_eqRGB));
        glAssert (glGetIntegerv (GL_BLEND_EQUATION_ALPHA, &_eqAlpha));
    }

    void restore() {
        if (_src != -1) {
            glAssert(glBlendEquationSeparate (_eqRGB, _eqAlpha));
            glAssert (glBlendFunc (_src, _dst));
        }
    }
};

// -----------------------------------------------------------------------------

/** the currently active matrix mode. */
class GLMatrixModeSave {
    GLint _mode;
public:
    GLMatrixModeSave () { save ();   }
    ~GLMatrixModeSave() { restore(); }

    void save   () { glAssert( glGetIntegerv(GL_MATRIX_MODE, &_mode) ); }
    void restore() { glAssert( glMatrixMode(_mode) );                   }
};

// -----------------------------------------------------------------------------

/** current clear colors */
class GLClearColorSave {
    float _color[4];
public:
    GLClearColorSave () { save();    }
    ~GLClearColorSave() { restore(); }

    void save   () { glAssert( glGetFloatv(GL_COLOR_CLEAR_VALUE, _color));   }
    void restore() {
        glAssert( glClearColor(_color[0], _color[1], _color[2], _color[3]));
    }
};

// -----------------------------------------------------------------------------

/** current clear colors */
class GLColorMaskSave {
    GLboolean _color[4];
public:
    GLColorMaskSave(GLboolean red, GLboolean green, GLboolean blue, GLboolean alpha) {
        save();
        glColorMask(red, green, blue, alpha);
    }
    GLColorMaskSave () { save();    }
    ~GLColorMaskSave() { restore(); }

    void save   () { glAssert( glGetBooleanv(GL_COLOR_WRITEMASK, _color));   }
    void restore() {
        glAssert( glColorMask(_color[0], _color[1], _color[2], _color[3]) );
    }
};
// -----------------------------------------------------------------------------

/** current clear depth */
class GLClearDepthSave {
    float _depth;

public:
    GLClearDepthSave  () { save();    }
    ~GLClearDepthSave () { restore(); }

    void save   () { glAssert( glGetFloatv(GL_DEPTH_CLEAR_VALUE, &_depth) ); }
    void restore() { glAssert( glClearDepth (_depth) );                      }
};

// -----------------------------------------------------------------------------

/** current clear stencil index */
class GLClearStencilSave {
    GLint _index;

public:
    GLClearStencilSave () { save();    }
    ~GLClearStencilSave() { restore(); }

    void save () { glAssert( glGetIntegerv(GL_STENCIL_CLEAR_VALUE, &_index) ); }
    void restore() { glAssert (glClearStencil (_index)); }
};

// -----------------------------------------------------------------------------

/** current depth mask */
class GLDepthMaskSave {
    GLboolean _depthMask;
public:
    GLDepthMaskSave (GLboolean s) {
        save();
        glDepthMask(s);
    }

    GLDepthMaskSave () { save();    }
    ~GLDepthMaskSave() { restore(); }

    void save   (){ glAssert( glGetBooleanv(GL_DEPTH_WRITEMASK, &_depthMask)); }
    void restore(){ glAssert( glDepthMask(_depthMask) );                       }
};

// -----------------------------------------------------------------------------

/** depth function */
class GLDepthFuncSave {
    GLint _depthFunc;
public:

    /// GL_NEVER, GL_LESS, GL_EQUAL, GL_LEQUAL,  GL_GREATER, GL_NOTEQUAL,
    /// GL_GEQUAL and GL_ALWAYS
    GLDepthFuncSave( GLint func) {
        save ();
        glAssert( glDepthFunc(func) );
    }

    GLDepthFuncSave () { save ();    }
    ~GLDepthFuncSave() { restore (); }

    void save   () { glAssert( glGetIntegerv(GL_DEPTH_FUNC, &_depthFunc) ); }
    void restore() { glAssert( glDepthFunc(_depthFunc) );                   }
};

// -----------------------------------------------------------------------------

/** cull face mode */
class GLCullFaceSave {
    GLint _cull;
public:

    GLCullFaceSave () { save ();   }
    ~GLCullFaceSave() { restore(); }

    void save   () { glAssert( glGetIntegerv (GL_CULL_FACE_MODE, &_cull) ); }
    void restore() { glAssert( glCullFace (_cull) );                        }
};

// -----------------------------------------------------------------------------

/** Save active texture unit */
class GLActiveTexUnitSave {
    GLint _texUnit;
public:

    GLActiveTexUnitSave() { save ();   }
    ~GLActiveTexUnitSave(){ restore(); }

    void save   () { glAssert( glGetIntegerv(GL_ACTIVE_TEXTURE, &_texUnit) ); }
    void restore() { glAssert( glActiveTexture(_texUnit)                   ); }
};

// -----------------------------------------------------------------------------

class GLPolygonModeSave {
    GLint _mode[2];

public:

    /// @param mode GL_LINE for wireframe or GL_FILL for plain faces
    GLPolygonModeSave(GLint mode) {
        save ();
        glAssert( glPolygonMode(GL_FRONT_AND_BACK, mode) );
    }
    GLPolygonModeSave() { save ();   }
    ~GLPolygonModeSave(){ restore(); }

    void save   () { glAssert( glGetIntegerv(GL_POLYGON_MODE, _mode   ) ); }
    void restore() {
        glAssert( glPolygonMode(GL_FRONT, _mode[0]) );
        glAssert( glPolygonMode(GL_BACK, _mode[1])  );
    }
};

// -----------------------------------------------------------------------------

/** texture modulation mode */
class GLTexEnvModeSave {
    GLint _texEnvMode;

public:
    GLTexEnvModeSave () { save();    }
    ~GLTexEnvModeSave() { restore(); }

    void save () {
        glAssert(glGetTexEnviv(GL_TEXTURE_ENV,
                               GL_TEXTURE_ENV_MODE,
                               &_texEnvMode));
    }

    void restore () {
        glAssert( glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, _texEnvMode) );
    }
};

// -----------------------------------------------------------------------------

class GLPolygonOffsetSave {
    GLfloat _factor;
    GLfloat _units;

public:
    GLPolygonOffsetSave( GLfloat factor, GLfloat units){
        save();
        glAssert( glPolygonOffset(factor, units) );
    }
    GLPolygonOffsetSave () { save();    }
    ~GLPolygonOffsetSave() { restore(); }

    void save () {
        glAssert(glGetFloatv(GL_POLYGON_OFFSET_FACTOR, &_factor));
        glAssert(glGetFloatv(GL_POLYGON_OFFSET_UNITS,  &_units) );
    }

    void restore () {
        glAssert( glPolygonOffset(_factor, _units) );
    }
};

// -----------------------------------------------------------------------------

/** DrawBuffer */
class GLDrawBufferSave {
private:
    GLint _drawBuffer;

public:
    GLDrawBufferSave (    ) { save ();          }
    GLDrawBufferSave (bool) { _drawBuffer = -1; }
    ~GLDrawBufferSave(    ) { restore ();       }

    void save    () { glAssert(glGetIntegerv(GL_DRAW_BUFFER, &_drawBuffer));   }
    void restore () { if(_drawBuffer!=-1) glAssert(glDrawBuffer(_drawBuffer)); }
};
// -----------------------------------------------------------------------------

class GLLineWidthSave {
    GLfloat _LineWidth;

public:
    GLLineWidthSave( GLfloat width ){
        save();
        glAssert( glLineWidth(width) );
    }

    GLLineWidthSave (    ) { save ();    }
    ~GLLineWidthSave(    ) { restore (); }

    void save    () { glAssert(glGetFloatv(GL_LINE_WIDTH, &_LineWidth));    }
    void restore () { if(_LineWidth!=-1.f) glAssert(glLineWidth(_LineWidth)); }
};

//------------------------------------------------------------------------------

class GLTextureBinding2DSave {
    GLint _BindedTex;

public:
    GLTextureBinding2DSave (    ) { save ();    }
    ~GLTextureBinding2DSave(    ) { restore (); }

    void save    () { glAssert(glGetIntegerv(GL_TEXTURE_BINDING_2D, &_BindedTex) ); }
    void restore () { glAssert(glBindTexture(GL_TEXTURE_2D, (GLuint)_BindedTex)  ); }
};

// -----------------------------------------------------------------------------

class GLViewportSave {
    GLint _Data[4];

public:
    GLViewportSave(GLint x, GLint y, GLsizei width, GLsizei height) {
        save ();
        glAssert( glViewport(x, y, width, height) );
    }
    GLViewportSave (    ) { save ();    }
    ~GLViewportSave(    ) { restore (); }

    void save    () { glAssert( glGetIntegerv(GL_VIEWPORT, _Data) );                  }
    void restore () { glAssert( glViewport(_Data[0], _Data[1], _Data[2], _Data[3]) ); }
};

// -----------------------------------------------------------------------------

class GLPointSizeSave {
    GLfloat _Size;

public:
    GLPointSizeSave ( GLfloat size ) {
        save ();
        glAssert( glPointSize(size) );
    }

    GLPointSizeSave () { save ();    }
    ~GLPointSizeSave() { restore (); }

    void save    () { glAssert( glGetFloatv(GL_POINT_SIZE, &_Size) ); }
    void restore () { glAssert( glPointSize(_Size)                 ); }
};

// -----------------------------------------------------------------------------


#endif

