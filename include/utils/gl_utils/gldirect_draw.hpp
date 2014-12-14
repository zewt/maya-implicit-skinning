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
#ifndef GL_DIRECT_DRAW_HPP__
#define GL_DIRECT_DRAW_HPP__

#include "glvao.hpp"
#include "glbuffer_object.hpp"
#include "shader.hpp"
#include <vector>
#include <map>

/**
 * @class GlDirect_draw
 * @brief A display list like utility but working with VBOs and VAOs
 *
 * Direct mode drawing and display list were kind of usefull for debugging.
 * Being able to quickly draw points, lines, triangles without taking care of
 * performances and ignoring GPU uploading issues was very convenient.
 *
 * GlDirect_draw provides these functionnalities using vertex buffer objects and
 * vertex array buffers.
 *
 * The logic stays almost the same as the old direct draw mode or display lists
 * with openGL legacy mode (2.1). It is compatible with newer openGL versions
 * such as the 4.0 because it does not actually use direct drawing.
 *
 * Here is how to draw a red triangle for instance:
 * @code
 *      GlDirect_draw prim;
 *
 *      prim.color3f(1.f, 0.f, 0.f);
 *      prim.begin(GL_TRIANGLES);
 *      prim.vertex3f(0.f, 0.f, 0.f);
 *      prim.vertex3f(0.f, 10.f, 0.f);
 *      prim.vertex3f(0.f, 0.f, 10.f);
 *      prim.end();
 *
 *      // Draw the triangle
 *      prim.set_matrix(model_view_mat, projection_mat);
 *      prim.draw();
 *      // release CPU GPU memory
 *      prim.clear();
 *      // Draw nothing
 *      prim.draw();
 * @endcode
 *
 * In addition to this "emulation" of the display list one can also update
 * specific attributes of a previously added vertex:
 * @code
 *      GlDirect_draw prim;
 *
 *      prim.color3f(1.f, 0.f, 0.f);
 *      prim.begin(GL_POINTS);
 *      GlDirect_draw::Attr_id i = prim.vertex3f(0.f, 0.f, 0.f);
 *      prim.end();
 *
 *      // Draw a red point at (0.f, 0.f, 0.f)
 *      prim.draw();
 *
 *      prim.begin_update(GL_POINTS);
 *      prim.set(i, ATTR_POSITION, 1.f, 1.f, 1.f      );
 *      prim.set(i, ATTR_COLOR   , 0.f, 1.f, 0.f, 0.5f);
 *      prim.end_update();
 *
 *      // Draw a blue point with alpha = 0.5f at (1.f, 1.f, 1.f)
 *      prim.draw();
 * @endcode
 *
 * Drawing use a static internal shader in GlDirect_draw. The shader is a
 * basic phong rendering. A call to draw() will activate the internal shader,
 * draw then restore the old active shader if it were any. Because we do not
 * rely on the old OpenGL global matrices the user need to specify the matrix
 * modelview and projection by hand with set_matrix().
 *
 * If you ever need to use a custom shader simply disable the internal one
 * and use yours like this:
 * @code
 *
 *      GlDirect_draw prim;
 *      // disable internal shader
 *      display_list.enable_internal_shader(false);
 *      // Specify indices of your custom shader linked to the attributes
 *      display_list.set_attr_index(GlDirect_draw::ATTR_POSITION, 0);
 *      display_list.set_attr_index(GlDirect_draw::ATTR_NORMAL  , 1);
 *
 *      prim.color3f(1.f, 0.f, 0.f);
 *      prim.begin(GL_TRIANGLES);
 *      prim.vertex3f(0.f, 0.f, 0.f);
 *      prim.vertex3f(0.f, 10.f, 0.f);
 *      prim.vertex3f(0.f, 0.f, 10.f);
 *      prim.end();
 *
 *      // Activate your custom shader
 *      glUseProgram( custom_shader_id );
 *      prim.set_matrix(model_view_mat, projection_mat);
 *      prim.draw();
 *      glUseProgram( 0 );
 *
 * @endcode
 *
 * If you wish for some reason to display primitives using OpenGL 2.1
 * and client state function family define the symbol USE_GL_LEGACY before
 * compiling
 *
 * @note This class is not intended for performances nor low memoy usage.
 * every attributes (position color etc.) are allocated in CPU/GPU wether
 * there are used or not. every pairs of begin() end() will result in a opengl
 * draw call and will create n == ATTR_SIZE VBOs and a VAO.
 * Use it only for small meshes or to debug. Keep the number of begin()
 * end() low (i.e keep them outside loops as much as possible)
 */

//#define USE_GL_LEGACY

class GlDirect_draw {
public:
    // =========================================================================
    /// @name Enums
    // =========================================================================

    /// @brief Supported attributes types
    enum Attr_t {
        ATTR_POSITION = 0,
        ATTR_NORMAL,
        ATTR_TEX_COORD,
        ATTR_COLOR,
        ATTR_SIZE,
        ATTR_CURRENTS
    };

private:
    /// @brief supported drawing modes. Replace 'MODE_' by 'GL_' to find the
    /// opengl drawing mode macro it corresponds to.
    enum Mode_t {
        MODE_POINTS = 0,
        MODE_LINE_STRIP,
        MODE_LINE_LOOP,
        MODE_LINES,
        MODE_TRIANGLE_FAN,
        MODE_TRIANGLES,
        MODE_QUADS,
        MODE_SIZE,
        MODE_ALL,
        MODE_NONE
    };

    // =========================================================================
    /// @name Inner struct
    // =========================================================================

    /// @brief holds attributes components (for position, normal, color etc.)
    struct Attr_data {
        Attr_data() {  size = -1; data[0] = data[1] = data[2] = data[3] = 0.f; }
        /// number of components (e.g: for a normal with (x,y,z) it would be 3  )
        int size;
        /// At maximum we can store 4 components
        float data[4];

        float& operator[](int i){ return data[i]; }

        void set(float x = 1.f, float y = 1.f, float z = 1.f, float w = 1.f){
            data[0] = x; data[1] = y; data[2] = z; data[3] = w;
        }
    };

    // -------------------------------------------------------------------------
public:

    /// @brief attribute identifier: it represents the location inside
    /// GlDirect_draw in order to update its value (for color, position etc.)
    /// @see vertex3f() set()
    struct Attr_id {
        Mode_t mode_t; ///< attribute drawing mode
        int buff_t;    ///< buffer index
        int idx;       ///< attribute index in the buffer
    };
    // =========================================================================
    /// @name Body
    // =========================================================================

    /// @param buffer_mode : hint used to managed vbo memory usage see
    /// opengl doc of glBufferData()
    GlDirect_draw(GLenum buffer_mode = GL_STATIC_DRAW);
    ~GlDirect_draw();

    /// Release all previously added attributes in GPU and CPU memory.
    /// Next call to draw() will have no effect.
    void clear();

    // =========================================================================
    /// @name Add attributes and primitives
    // =========================================================================

    /// Begin to add primitives to draw in GPU memory.
    void begin( GLenum gl_mode );

    /// Add the vertex attributes. Attributes values matches the last calls
    /// of color3f() normal3f() etc.
    /// @warning this call must be done inside begin() end() calls
    Attr_id vertex3f(GLfloat x, GLfloat y, GLfloat z);

    /// Stops adding primitives in GPU memory
    /// @param direct_draw : wether added primitives since the last begin()
    /// should been drawn.
    void end(bool direct_draw = false);

    // =========================================================================
    /// @name Set attributes active values
    // =========================================================================

    /// sets the current value of the color attribute
    /// @note this may be called any time
    void color3f (GLfloat r, GLfloat g, GLfloat b);

    /// sets the current value of the color attribute
    /// @note this may be called any time
    void color4f (GLfloat r, GLfloat g, GLfloat b, GLfloat a);

    /// sets the current value of the normal attribute
    /// @note this may be called any time
    void normal3f(GLfloat x, GLfloat y, GLfloat z);

    /// sets the current value of the texture coordinate attribute
    /// @note this may be called any time
    void texCoords2f(GLfloat u, GLfloat v);

    // =========================================================================
    /// @name Modify primitives
    // =========================================================================

    /// Enable updating already specified attributes.
    /// @param gl_mode : the drawing mode you want to update. If you want to
    /// update all the modes at the same time you can set it to GL_FALSE.
    /// However the update will get slower in this case
    void begin_update(GLenum gl_mode = GL_FALSE);

    /// Change the values of an attribute
    /// @param id : identifier of the attribute previously created with
    /// vertex3f().
    /// @param type : the attribute type to change (position, color etc.).
    /// if type == ATTR_CURRENTS then (x, y, z, w) will be interpreted as the
    /// position and the other attributes will be assigned the last active
    /// values sets by color3f(), normal3f() likewise functions.
    /// @param x,y,z,w : the attribute new values. Only the relevant components
    /// will be modified (e.g (z,w) will be ignored for type == ATTR_TEX_COORD).
    /// @warning this call must be done inside begin_update() end_update() calls
    void set(const Attr_id& id,
             Attr_t type,
             GLfloat x = 0.f, GLfloat y = 0.f, GLfloat z = 0.f, GLfloat w = 0.f);
    /// Stop changing values
    void end_update();

    // =========================================================================
    /// @name Draw
    // =========================================================================

    /// Draw the previously added attribute within begin() end() calls.
    /// @warning should be call outside a pair of begin() end() or
    /// begin_update() end_update()
    void draw();

    /// Set the matrices use to draw with the internal shader.
    /// @param model_view, proj : matrix of modelView and projection in OpenGL
    /// format (i.e. column major)
    void set_matrix(const float model_view[16], const float proj[16] );

    // Default : value (1.f, 1.f, 1.f)
    //void set_light_color(float r, float g, float b);
    // Default value : true
    //void enable_internal_shader(bool state);
    // Default value : true
    //void enable_lighting(bool state);
    // Default value : false
    //void enable_texture(bool state, );

    /// disable internal shader to enable the use of a custom shader
    /// when state is false you can call draw() using your own shader
    /// @note default vaule is true
    /// @warning don't forget to set the index of attributes
    void enable_internal_shader(bool state) { _use_int_shader = state; }

    /// Sets the VAO attribute index for the shader use -1 to disable the
    /// attribute. By default all attributes are disabled
    /// @note indices will be take into accout for the next pair of begin()
    /// end()
    void set_attr_index(Attr_t attr, int index){ _attrs_index[attr] = index; }


    // =========================================================================
    /// @name Getters & Setters
    // =========================================================================

    /// @note Not implemented yet
    /// TODO: automatically compute flat normals for triangular elements
    void set_auto_normals  (bool s){ _auto_normals   = s; }

    /// Automatically normalize the normal attribute
    void set_auto_normalize(bool s){ _auto_normalize = s; }

    // * vao(); get the vao

private:
    // =========================================================================
    /// @name tools
    // =========================================================================

    GLenum our_mode_to_gl_mode(Mode_t mode);

    /// draw the ith buffer given its mode.
    void draw_buffer(Mode_t mode_t, int i);

    /// Prepare gl states to use the internal direct draw shader
    void begin_shader();
    /// Restor gl states
    void end_shader();

    // =========================================================================
    /// @name Class attributes
    // =========================================================================

    GLenum _buffer_mode; ///< hint to create VBO

    /// Attribute associated index for drawing with shaders
    int _attrs_index[ATTR_SIZE];

    bool _use_int_shader; ///< wether we use internal shader to draw primitives
    bool _is_mat_set;     ///< wether matrices has been set for display
    bool _is_begin;       ///< wether we are between begin() end() or not
    bool _is_update;      ///< wether we are between begin_update() end_update() or not
    bool _auto_normals;   ///< automatic computation of normals
    bool _auto_normalize; ///< automatic normalisation

    Mode_t    _curr_mode;             ///< curent drawing mode
    Attr_data _attributes[ATTR_SIZE]; ///< current attributes value

    /// Converts openGL enums to our supported drawing type
    std::map<GLenum, Mode_t> _gl_mode_to_our;

    /// For each attribute (Vertex, normals, etc.) we store drawing modes
    /// (Triangles, Quads etc.).
    /// Each drawing mode stores a list of buffers corresponding to every
    /// begin() end() calls
    /// _vertices[Attribute type][drawing mode][list of buffers][datas (vertex or color or ... components)]

    /// GPU storage of the attributes (position, normals etc.)
    std::vector< GlBuffer_obj* > _gpu_buffers[ATTR_SIZE][MODE_SIZE];

    /// Vertex array buffer for each group of buffer object
    std::vector< GlVao* > _vaos[MODE_SIZE];

    /// GPU mapped pointers on cpu (mapping effective between begin_update() and
    /// end_update() )
    std::vector<float*> _gpu_maps[ATTR_SIZE][MODE_SIZE];

    /// CPU storage of the attributes (position, normals etc.)
    std::vector< std::vector<float> > _cpu_buffers[ATTR_SIZE][MODE_SIZE];

    GLint _prev_shader; ///< saved shader id by begin_shader()
};

#endif // GL_DIRECT_DRAW_HPP__
