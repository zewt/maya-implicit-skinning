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
#include "gldirect_draw.hpp"

#include <iostream>
#include <cmath>
#include "shader.hpp"

#ifndef NDEBUG
#define assert_msg(a, str)                  \
while(0){                                   \
    if(!(a)){                               \
        std::cerr << (str) << std::endl;    \
        assert(false);                      \
    }                                       \
}

#else
#define assert_msg(a, str)
#endif

// -----------------------------------------------------------------------------

// Copy/paste procedure from the MESA inplem of glu
// and added the template
template<typename T>
static bool invertMatrix(const T m[16], T invOut[16])
{
    T inv[16], det;
    int i;

    inv[ 0] =  m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
    inv[ 4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
    inv[ 8] =  m[4] * m[ 9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[ 9];
    inv[12] = -m[4] * m[ 9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[ 9];
    inv[ 1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
    inv[ 5] =  m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
    inv[ 9] = -m[0] * m[ 9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[ 9];
    inv[13] =  m[0] * m[ 9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[ 9];
    inv[ 2] =  m[1] * m[ 6] * m[15] - m[1] * m[ 7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[ 7] - m[13] * m[3] * m[ 6];
    inv[ 6] = -m[0] * m[ 6] * m[15] + m[0] * m[ 7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[ 7] + m[12] * m[3] * m[ 6];
    inv[10] =  m[0] * m[ 5] * m[15] - m[0] * m[ 7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[ 7] - m[12] * m[3] * m[ 5];
    inv[14] = -m[0] * m[ 5] * m[14] + m[0] * m[ 6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[ 6] + m[12] * m[2] * m[ 5];
    inv[ 3] = -m[1] * m[ 6] * m[11] + m[1] * m[ 7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[ 9] * m[2] * m[ 7] + m[ 9] * m[3] * m[ 6];
    inv[ 7] =  m[0] * m[ 6] * m[11] - m[0] * m[ 7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[ 8] * m[2] * m[ 7] - m[ 8] * m[3] * m[ 6];
    inv[11] = -m[0] * m[ 5] * m[11] + m[0] * m[ 7] * m[ 9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[ 9] - m[ 8] * m[1] * m[ 7] + m[ 8] * m[3] * m[ 5];
    inv[15] =  m[0] * m[ 5] * m[10] - m[0] * m[ 6] * m[ 9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[ 9] + m[ 8] * m[1] * m[ 6] - m[ 8] * m[2] * m[ 5];

    det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];

    if(det == 0)
        return false;

    det = (T)1 / det;

    for(i = 0; i < 16; i++)
        invOut[i] = inv[i] * det;

    return true;
}

// -----------------------------------------------------------------------------

// Copy/paste procedure from the MESA inplem of glu
// and added the template
template<typename T>
static void multMatrices(const T a[16],
                         const T b[16],
                         T r[16])
{
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            r[i*4+j] =
                    a[i*4+0] * b[0*4+j] +
                    a[i*4+1] * b[1*4+j] +
                    a[i*4+2] * b[2*4+j] +
                    a[i*4+3] * b[3*4+j];
        }
    }
}

// -----------------------------------------------------------------------------

template<typename T>
static void transposeMatrix(const T a[16], T r[16])
{
    T tr[16];
    int i, j;
    for (i = 0; i < 4; i++) {
        for (j = 0; j < 4; j++) {
            tr[i*4 + j] = a[j*4 + i];
        }
    }

    for(i = 0; i < 16; ++i)
        r[i] = tr[i];
}


// -----------------------------------------------------------------------------

/**
  @namespace Shader_dd
  @brief Encapsulate internal shaders used by GlDirect_draw

  Simple hard coded phong shader used to draw with GlDirect_draw.
  Handles a single light attached to the camera
*/
// =============================================================================
namespace Shader_dd {
// =============================================================================

Shader_prog* phong_shader = 0;
int acc = 0;

// -----------------------------------------------------------------------------

// Hard coded phong vertex shader
const char* src_vert =
        "#version 150\n"
        "// Matrices\n"
        "uniform mat4 modelViewMatrix;\n"
        "uniform mat4 projectionMatrix;\n"
        "uniform mat4 MVP;\n"
        "uniform mat4 normalMatrix;\n"
        "in vec3 inPosition;\n"
        "in vec3 inNormal;\n"
        "in vec4 inTexCoord;\n"
        "in vec4 inColor;\n"
        "// Sommet\n"
        "out vec3 varNormal;\n"
        "out vec4 varTexCoord;\n"
        "out vec4 varColor;\n"
        "// Eclairage\n"
        "out vec3 lightDirInView;\n"
        "out vec3 halfVecInView;\n"
        "void computeLightingVectorsInView(in vec3 posInView, in vec3 lightPosition, out vec3 lightDir, out vec3 halfVec){\n"
        "   lightDir = normalize(lightPosition-posInView);\n"
        "   vec3 viewDir = normalize(vec3(0.0, 0.0, 0.0) - posInView);\n"
        "   halfVec = normalize (lightDir + viewDir);\n"
        "}\n"
        "void main(void) {\n"
        "   vec3 posInView = vec3( modelViewMatrix * vec4(inPosition.xyz, 1.0));\n"
        "   computeLightingVectorsInView(posInView, vec3(0,0,0), lightDirInView, halfVecInView);\n"
        "   varNormal = normalize((normalMatrix * vec4(inNormal,0.0)).xyz);\n"
        "   varTexCoord = inTexCoord;\n"
        "   gl_Position = MVP*vec4(inPosition.xyz, 1.0);\n"
        "   varColor = inColor;\n"
        "}\n\0";

// -----------------------------------------------------------------------------

// Hard coded phong fragment shader
const char* src_frag =
        "#version 150\n"
        "// material\n"
        "uniform vec3 materialKd;\n"
        "uniform vec3 materialKs;\n"
        "uniform float materialNs;\n"
        "// fragment\n"
        "in vec3 varNormal;\n"
        "in vec4 varTexCoord;\n"
        "in vec4 varColor;\n"
        "// lights\n"
        "in vec3 lightDirInView;\n"
        "in vec3 halfVecInView;\n"
        "// material\n"
        "uniform vec3 lightColor;\n"
        "// resultat\n"
        "out vec4 outColor;\n"
        "vec3 blinnPhongLighting (in vec3 kd, in vec3 ks, in float ns, in vec3 normal, in vec3 lightVector, in vec3 halfVector){\n"
        "   float costetha = clamp(dot(normal, lightVector), 0., 1.);\n"
        "   float cosalphan = pow( clamp( dot(halfVector,normal), 0., 1.), ns );\n"
        "   vec3 result = (kd + (ks* cosalphan)) * costetha;\n"
        "   return result;\n"
        "}\n"
        "void main(void) {\n"
        "   #ifdef LIGHTING_ENABLE \n"
        "      vec3 fragColor = vec3(0.0, 0.0, 0.0);\n"
        "      fragColor = lightColor * blinnPhongLighting(materialKd, materialKs, materialNs, normalize(varNormal), normalize(lightDirInView), normalize(halfVecInView));\n"
        "      outColor = vec4( fragColor*0.9, 1.);\n"
        "   #else \n"
        "       outColor = varColor;\n"
        "   #endif \n"
        "}\n\0";

// -----------------------------------------------------------------------------

void init()
{
    if( acc > 0 ) return;
    acc++;

    Shader vertex(GL_VERTEX_SHADER  );
    Shader frag  (GL_FRAGMENT_SHADER);

    if( !vertex.load_source( src_vert ) ) assert(false);
    if( !frag.  load_source( src_frag ) ) assert(false);

    phong_shader = new Shader_prog(vertex, frag);
    phong_shader->bind_attribute("inPosition", GlDirect_draw::ATTR_POSITION );
    phong_shader->bind_attribute("inNormal"  , GlDirect_draw::ATTR_NORMAL   );
    phong_shader->bind_attribute("inTexCoord", GlDirect_draw::ATTR_TEX_COORD);
    phong_shader->bind_attribute("inColor"   , GlDirect_draw::ATTR_COLOR    );
    phong_shader->link();

    if( !phong_shader->get_status() ) assert(false);
}

// -----------------------------------------------------------------------------

void clear()
{
    acc--;
    if( acc > 0) return;
    delete phong_shader;
    phong_shader = 0;
}

}// END SHADER_DD ==============================================================

// -----------------------------------------------------------------------------

GlDirect_draw::GlDirect_draw(GLenum buffer_mode) :
    _buffer_mode(buffer_mode),
    _use_int_shader(true),
    _is_mat_set(false),
    _is_begin(false),
    _is_update(false),
    _auto_normals(false),
    _auto_normalize(false),
    _curr_mode(MODE_NONE)
{
    for (int i = 0; i < ATTR_SIZE; ++i) _attrs_index[i] = -1;

    // Init attributes component size
    _attributes[ATTR_POSITION ].size = 3; // x, y, z
    _attributes[ATTR_NORMAL   ].size = 3; // x, y, z
    _attributes[ATTR_TEX_COORD].size = 2; // u, v
    _attributes[ATTR_COLOR    ].size = 4; // r, g, b, a

    // Map openGL enums to ours
    _gl_mode_to_our[GL_POINTS]       = MODE_POINTS;
    _gl_mode_to_our[GL_LINE_STRIP]   = MODE_LINE_STRIP;
    _gl_mode_to_our[GL_LINE_LOOP]    = MODE_LINE_LOOP;
    _gl_mode_to_our[GL_LINES]        = MODE_LINES;
    _gl_mode_to_our[GL_TRIANGLE_FAN] = MODE_TRIANGLE_FAN;
    _gl_mode_to_our[GL_TRIANGLES]    = MODE_TRIANGLES;
    _gl_mode_to_our[GL_QUADS]        = MODE_QUADS;

    Shader_dd::init();
}

// -----------------------------------------------------------------------------

void GlDirect_draw::clear()
{
    for (int attr_t = 0; attr_t < ATTR_SIZE; ++attr_t){
        for (int mode_t = 0; mode_t < MODE_SIZE; ++mode_t){
            int size = (int)_gpu_buffers[attr_t][mode_t].size();
            for (int ith_buffer = 0; ith_buffer < size; ++ith_buffer)
                delete _gpu_buffers[attr_t][mode_t][ith_buffer];

            _gpu_buffers[attr_t][mode_t].clear();
            _gpu_maps   [attr_t][mode_t].clear();
            _cpu_buffers[attr_t][mode_t].clear();
        }
    }

    // Delete Vaos:
    for (int mode_t = 0; mode_t < MODE_SIZE; ++mode_t){
        int size = (int)_vaos[mode_t].size();
        for (int ith_vao = 0; ith_vao < size; ++ith_vao)
            delete _vaos[mode_t][ith_vao];
        _vaos[mode_t].clear();
    }
}

// -----------------------------------------------------------------------------

GlDirect_draw::~GlDirect_draw()
{
    clear();
    Shader_dd::clear();
}

// -----------------------------------------------------------------------------

void GlDirect_draw::begin( GLenum gl_mode )
{
    assert_msg( !_is_update, "ERROR: can't be called inside begin_update() end_update() calls");
    assert_msg(!_is_begin, "ERROR: imbricated begin() end() are forbidden");
    _is_begin = true;

    std::map<GLenum, Mode_t>::iterator it = _gl_mode_to_our.find( gl_mode );
    assert_msg( it != _gl_mode_to_our.end(), "ERROR: unsupported drawing mode");
    _curr_mode = it->second;

    for(int attr_t = 0; attr_t < ATTR_SIZE; ++attr_t){
        _cpu_buffers[attr_t][_curr_mode].push_back( std::vector<float>()                );
        _gpu_buffers[attr_t][_curr_mode].push_back( new GlBuffer_obj(GL_ARRAY_BUFFER)   );
        _gpu_maps   [attr_t][_curr_mode].push_back( 0                                   );
    }
    _vaos[_curr_mode].push_back( new GlVao() );
}

// -----------------------------------------------------------------------------

GlDirect_draw::Attr_id GlDirect_draw::vertex3f(GLfloat x, GLfloat y, GLfloat z)
{
    assert_msg(!_is_update, "ERROR: can't be called inside begin_update() end_update() calls");
    assert_msg(_is_begin, "ERROR: vertex3f() must be called between begin() end() calls");

    _attributes[ATTR_POSITION].set(x, y, z, 1.f);

    int plast = (int)_cpu_buffers[ATTR_POSITION][_curr_mode].size() - 1;

    for(int attr_t = 0; attr_t < ATTR_SIZE; ++attr_t)
    {
        int last = (int)_cpu_buffers[attr_t][_curr_mode].size() - 1;
        assert(last >= 0    );
        assert(last == plast);

        std::vector<float>& buff = _cpu_buffers[attr_t][_curr_mode][last];

        // Copy components (x, y, z ...)
        for(int comp = 0; comp < _attributes[attr_t].size; ++comp)
            buff.push_back(_attributes[attr_t][comp]);
    }

    int idx    = (_cpu_buffers[ATTR_POSITION][_curr_mode][plast].size() - 1) / _attributes[ATTR_POSITION].size;
    Attr_id id = { _curr_mode, plast, idx };
    return id;
}

// -----------------------------------------------------------------------------

void GlDirect_draw::color3f(GLfloat r, GLfloat g, GLfloat b)
{
    _attributes[ATTR_COLOR].set(r, g, b, 1.f);
}

// -----------------------------------------------------------------------------

void GlDirect_draw::color4f(GLfloat r, GLfloat g, GLfloat b, GLfloat a)
{
    _attributes[ATTR_COLOR].set(r, g, b, a);
}

// -----------------------------------------------------------------------------

void GlDirect_draw::normal3f(GLfloat x, GLfloat y, GLfloat z)
{
    if( _auto_normalize)
    {
        float n = sqrtf(x*x + y*y + z*z);
        x /= n; y /= n; z /= n;
    }
    _attributes[ATTR_NORMAL].set(x, y, z, 0.f);
}

// -----------------------------------------------------------------------------

void GlDirect_draw::texCoords2f(GLfloat u, GLfloat v)
{
    _attributes[ATTR_COLOR].set(u, v, 0.f, 0.f);
}

// -----------------------------------------------------------------------------

void GlDirect_draw::end(bool direct_draw)
{
    assert_msg( !_is_update, "ERROR: can't be called inside begin_update() end_update() calls");
    assert_msg( _is_begin  , "ERROR: imbricated begin() end() are forbidden");
    _is_begin = false;

    int plast = (int)_cpu_buffers[ATTR_POSITION][_curr_mode].size() - 1;
    int last  = (int)_vaos[_curr_mode].size() - 1;

    _vaos[_curr_mode][last]->bind();
    assert( plast == last);
    // Upload to GPU:
    // N.B: we recopy everything ( even previous begin() end() calls)
    for(int attr_t = 0; attr_t < ATTR_SIZE; ++attr_t)
    {
        int last = (int)_cpu_buffers[attr_t][_curr_mode].size() - 1;
        assert(last >= 0    );
        assert(last >= plast); // <- why not equal

        std::vector<float>& in  = _cpu_buffers[attr_t][_curr_mode][last];
        GlBuffer_obj*       out = _gpu_buffers[attr_t][_curr_mode][last];

        out->set_data(in.size(), &(in[0]), GL_STATIC_DRAW);
        int idx_attr = _use_int_shader ? attr_t : _attrs_index[attr_t];
        if(idx_attr > -1)
            _vaos[_curr_mode][last]->record_attr(out->get_id(), idx_attr, _attributes[attr_t].size);
    }
    _vaos[_curr_mode][last]->unbind();

    if(direct_draw){
        begin_shader();
        draw_buffer(_curr_mode, plast);
        end_shader();
    }

    _curr_mode = MODE_NONE;
}

// -----------------------------------------------------------------------------

void GlDirect_draw::begin_update( GLenum gl_mode )
{
    assert_msg( !_is_begin , "ERROR: can't be called inside begin() end() calls");
    assert_msg( !_is_update, "ERROR: imbricated begin_update() end_update() are forbidden");
    _is_update = true;

    int start_mode = 0;
    int end_mode   = MODE_SIZE;

    if( gl_mode == GL_FALSE ){
        // Map every modes
        _curr_mode = MODE_ALL;
    } else {
        // Only map the activated mode
        std::map<GLenum, Mode_t>::iterator it = _gl_mode_to_our.find( gl_mode );
        assert_msg( it != _gl_mode_to_our.end(), "ERROR: unsupported drawing mode");
        _curr_mode = it->second;
        start_mode = _curr_mode;
        end_mode   = start_mode + 1;
    }

    for (int attr_t = 0; attr_t < ATTR_SIZE; ++attr_t){
        for (int mode_t = start_mode; mode_t < end_mode; ++mode_t){
            int size = (int)_gpu_buffers[attr_t][mode_t].size();
            for (int ith_buffer = 0; ith_buffer < size; ++ith_buffer){
                float* m = 0;
                _gpu_buffers[attr_t][mode_t][ith_buffer]->map_to(m, GL_WRITE_ONLY);
                assert( m != 0); // Can't map the vbo apparently
                _gpu_maps   [attr_t][mode_t][ith_buffer] = m;
            }
        }
    }
}

// -----------------------------------------------------------------------------

void GlDirect_draw::set(const Attr_id& v, Attr_t type, GLfloat x, GLfloat y, GLfloat z, GLfloat w)
{
    assert_msg( !_is_begin , "ERROR: can't be called inside begin() end() calls");
    assert_msg( _is_update,
                "ERROR: set() must be called between begin_update() end_update() calls");
    assert_msg(_curr_mode == MODE_ALL || v.mode_t == _curr_mode,
               "ERROR: trying to update an attribute with a drawing mode different from the current one.");

    int attr_t = 0;
    int end_attr = ATTR_SIZE;
    if(type != ATTR_CURRENTS){
        attr_t   = type;
        end_attr = type + 1;
        _attributes[type].set(x, y, z, w);
    }else
        _attributes[ATTR_POSITION].set(x, y, z, 1.f);

    for (; attr_t < end_attr; ++attr_t){
        for(int i = 0; i < _attributes[attr_t].size; ++i){
            const int idx = v.idx * _attributes[attr_t].size + i;
            assert(_gpu_maps[attr_t][v.mode_t][v.buff_t] != 0); // The VBO is not mapped ?
            _gpu_maps[attr_t][v.mode_t][v.buff_t][idx] = _attributes[attr_t][i];
        }
    }
}

// -----------------------------------------------------------------------------

void GlDirect_draw::end_update()
{
    assert_msg( !_is_begin , "ERROR: can't be called inside begin() end() calls");
    assert_msg( _is_update, "ERROR: imbricated begin_update() end_update() are forbidden");
    _is_update = false;

    int start_mode = 0;
    int end_mode   = MODE_SIZE;

    if( _curr_mode != MODE_ALL ){
        // Only unmap the activated mode
        start_mode = _curr_mode;
        end_mode   = start_mode + 1;
    }

    for (int attr_t = 0; attr_t < ATTR_SIZE; ++attr_t){
        for (int mode_t = start_mode; mode_t < end_mode; ++mode_t){
            int size = (int)_gpu_buffers[attr_t][mode_t].size();
            for (int ith_buffer = 0; ith_buffer < size; ++ith_buffer){
                _gpu_buffers[attr_t][mode_t][ith_buffer]->unmap();
                _gpu_maps   [attr_t][mode_t][ith_buffer] = 0;
            }
        }
    }

    // unbind vbos
    glAssert( glBindBuffer(GL_ARRAY_BUFFER, 0) );

    _curr_mode = MODE_NONE;
}

// -----------------------------------------------------------------------------

void GlDirect_draw::draw()
{
    assert_msg( !_is_begin , "ERROR: can't draw inside begin() end() calls");
    assert_msg( !_is_update, "ERROR: can't be called inside begin_update() end_update() calls");

    begin_shader();

    // for each mode (GL_TRIANGLES, GL_LINE_STRIP etc.)
    for (int mode_t = 0; mode_t < MODE_SIZE; ++mode_t)
    {        // Look up associated buffers
        int s = (int)_gpu_buffers[ATTR_POSITION][mode_t].size();
        for (int i = 0; i < s; ++i)
            draw_buffer((Mode_t)mode_t, i);
    }

    end_shader();
}

// -----------------------------------------------------------------------------

void GlDirect_draw::set_matrix(const float model_view[16],
                               const float proj[16] )
{
    float MVP[16];
    float normal_mat[16];

    multMatrices( model_view, proj, MVP);
    invertMatrix(MVP, normal_mat);
    transposeMatrix(normal_mat, normal_mat);

    GLint sh_id = 0;
    glAssert( glGetIntegerv(GL_CURRENT_PROGRAM, &sh_id) );
    Shader_dd::phong_shader->use();

    Shader_dd::phong_shader->set_mat4x4("MVP", MVP);
    Shader_dd::phong_shader->set_mat4x4("normalMatrix", normal_mat);
    Shader_dd::phong_shader->set_mat4x4("modelViewMatrix", model_view);
    Shader_dd::phong_shader->set_mat4x4("projectionMatrix", proj);

    if(sh_id >= 0) glAssert( glUseProgram( sh_id ) );
    _is_mat_set = true;
}

// -----------------------------------------------------------------------------

void GlDirect_draw::draw_buffer(Mode_t mode_t, int i)
{
    // empty buffer skip it
    if( _gpu_buffers[ATTR_POSITION][mode_t][i]->size() == 0)
    {
        std::cerr << "WARNING: empty vbo, maybe you didn't put";
        std::cerr << " a vertex3f() between begin() end() calls";
        std::cerr << std::endl;
        return;
    }

#ifndef USE_GL_LEGACY
    assert(_is_mat_set);
    // Opengl 3.1 and superior drawing
    _vaos[mode_t][i]->bind();

    ///////////////////
    // OpenGl draw call
    unsigned nb_elts   = _cpu_buffers[ATTR_POSITION][mode_t][i].size();
    unsigned size_comp = (int)_attributes[ATTR_POSITION].size;
    assert(  nb_elts % size_comp == 0 );

    GLenum gl_mode = our_mode_to_gl_mode( (Mode_t)mode_t );
    int nb_verts = (nb_elts / size_comp);
    glAssert( glDrawArrays(gl_mode, 0,  nb_verts) );

    _vaos[mode_t][i]->unbind();
#else
    // Opengl legacy (2.1) drawing
    // Activate each attribute in the current buffer

    ////////////////////////
    // Enable client states

    // Enable position
    _gpu_buffers[ATTR_POSITION][mode_t][i]->bind();
    glAssert( glEnableClientState(GL_VERTEX_ARRAY) );
    glAssert( glVertexPointer(_attributes[ATTR_POSITION].size, GL_FLOAT, 0, 0) );

    // Enable normal
    _gpu_buffers[ATTR_NORMAL][mode_t][i]->bind();
    glAssert( glEnableClientState(GL_NORMAL_ARRAY) );
    glAssert( glNormalPointer(GL_FLOAT, 0, 0) );

    // Enable texture coordinates
    _gpu_buffers[ATTR_TEX_COORD][mode_t][i]->bind();
    glAssert( glEnableClientState(GL_TEXTURE_COORD_ARRAY) );
    glAssert( glTexCoordPointer(_attributes[ATTR_TEX_COORD].size, GL_FLOAT, 0, 0) );

    // Enable color
    _gpu_buffers[ATTR_COLOR][mode_t][i]->bind();
    glAssert( glEnableClientState(GL_COLOR_ARRAY) );
    glAssert( glColorPointer(_attributes[ATTR_COLOR].size, GL_FLOAT, 0, 0) );

    ///////////////////
    // OpenGl draw call
    unsigned nb_elts   = _cpu_buffers[ATTR_POSITION][mode_t][i].size();
    unsigned size_comp = (int)_attributes[ATTR_POSITION].size;
    assert(  nb_elts % size_comp == 0 );

    GLenum gl_mode = our_mode_to_gl_mode( (Mode_t)mode_t );
    int nb_verts = (nb_elts / size_comp);
    glAssert( glDrawArrays(gl_mode, 0,  nb_verts) );

    ////////////////////////
    // Disable client states
    glAssert( glBindBuffer(GL_ARRAY_BUFFER, 0) );
    glAssert( glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0) );

    glAssert( glDisableClientState(GL_VERTEX_ARRAY) );
    glAssert( glDisableClientState(GL_NORMAL_ARRAY) );
    glAssert( glDisableClientState(GL_TEXTURE_COORD_ARRAY) );
    glAssert( glDisableClientState(GL_COLOR_ARRAY) );

    glAssert( glVertexPointer(_attributes[ATTR_POSITION].size, GL_FLOAT, 0, 0) );
    glAssert( glNormalPointer(GL_FLOAT, 0, 0) );
    glAssert( glTexCoordPointer(_attributes[ATTR_TEX_COORD].size, GL_FLOAT, 0, 0) );
    glAssert( glColorPointer(_attributes[ATTR_COLOR].size, GL_FLOAT, 0, 0) );
#endif
}


// -----------------------------------------------------------------------------

GLenum GlDirect_draw::our_mode_to_gl_mode(Mode_t mode)
{
    std::map<GLenum, Mode_t>::iterator it = _gl_mode_to_our.begin();
    for (; it != _gl_mode_to_our.end(); ++it) {
        if( it->second == mode)
            return it->first;
    }
    return -1;
}

// -----------------------------------------------------------------------------

void GlDirect_draw::begin_shader()
{
#ifndef USE_GL_LEGACY
    if( !_use_int_shader ) return;
    glAssert( glGetIntegerv(GL_CURRENT_PROGRAM, &_prev_shader) );
    Shader_dd::phong_shader->use();

    Shader_dd::phong_shader->set_uniform("materialKd", 1.f, 1.f, 1.f);
    Shader_dd::phong_shader->set_uniform("materialKs", 0.f, 0.f, 0.f);
    Shader_dd::phong_shader->set_uniform("materialNs", 1.f);
    Shader_dd::phong_shader->set_uniform("lightColor", 1.f, 1.f, 1.f);
    assert_msg(_attrs_index[ATTR_POSITION] > -1, "ERROR: you forgot to setup the attribute index");
#endif
}

// -----------------------------------------------------------------------------

void GlDirect_draw::end_shader()
{
#ifndef USE_GL_LEGACY
    if( !_use_int_shader ) return;
    if(_prev_shader >= 0) glAssert( glUseProgram( _prev_shader ) );
#endif
}

// -----------------------------------------------------------------------------


// =============================================================================
// Some Unit test
// =============================================================================
#if 0

#include "gldirect_draw.hpp"

void test()
{
    GLEnabledSave depth   (GL_DEPTH_TEST, true, true );
    GLEnabledSave lighting(GL_LIGHTING  , true, false);
    GLEnabledSave blend   (GL_BLEND     , true, false);
    GLEnabledSave texture (GL_TEXTURE_2D, true, false);

    GLLineWidthSave line_width(5.f);
    GLPointSizeSave point_size(5.f);

    bool d = true;
    for (int i = 0; i < 2; ++i)
    {
        float off = i*20;
        GlDirect_draw prim;

        GLfloat mv[16], proj[16];
        glGetFloatv(GL_MODELVIEW_MATRIX, mv);
        glGetFloatv(GL_PROJECTION_MATRIX, proj);
        prim.set_matrix( mv, proj);

        prim.color3f(1.f, 0.f, 0.f);

        prim.begin(GL_TRIANGLES);
        prim.vertex3f(off+0.f, 0.f, 0.f);
        prim.vertex3f(off+0.f, 10.f, 0.f);
        prim.vertex3f(off+0.f, 0.f, 10.f);
        prim.end(d);

        prim.color3f(1.f, 1.f, 0.f);
        prim.begin(GL_POINTS);
        prim.vertex3f(off+0.f, 0.f, 0.f);
        prim.vertex3f(off+0.f, 10.f, 0.f);
        prim.vertex3f(off+0.f, 0.f, 10.f);
        prim.end(d);

        prim.color3f(1.f, 0.f, 1.f);
        prim.begin(GL_LINES);
        prim.vertex3f(off+0.f, 0.f, 0.f);
        prim.vertex3f(off+0.f, 10.f, 10.f);
        prim.vertex3f(off+0.f, 10.f, 0.f);
        prim.vertex3f(off+0.f, 0.f, 10.f);
        prim.end(d);

        prim.begin(GL_LINES);
        prim.color3f(0.5f, 1.f, 0.f);
        prim.vertex3f(off+0.f, 0.f, 0.f);
        prim.color3f(0.5f, 1.f, 0.5f);
        prim.vertex3f(off+10.f, 0.f, 0.f);

        prim.color3f(0.f, 1.f, 0.f);
        prim.vertex3f(off+0.f, 0.f, 0.f);
        prim.color3f(0.5f, 0.f, 1.f);
        prim.vertex3f(off+0.f, 0.f, 10.f);
        prim.end(d);

        prim.color3f(0.f, 1.f, 0.f);
        prim.begin(GL_LINE_STRIP);
        prim.vertex3f(off+-5.f, 0.f, 0.f);
        prim.vertex3f(off+-5.f, 10.f, 10.f);
        prim.vertex3f(off+-5.f, 10.f, 0.f);
        prim.vertex3f(off+-5.f, 0.f, 10.f);
        prim.end(d);

        prim.color3f(0.f, 0.f, 1.f);
        prim.begin(GL_LINE_STRIP);
        prim.vertex3f(off+-10.f, 0.f, 0.f  );
        prim.vertex3f(off+-10.f, 10.f, 10.f);
        prim.vertex3f(off+-10.f, 10.f, 0.f );
        prim.vertex3f(off+-10.f, 0.f, 10.f );
        prim.end(d);

        prim.begin(GL_LINE_LOOP);
        prim.vertex3f(off+0.f,  0.f, 20.f  );
        prim.vertex3f(off+0.f, 10.f, 20.f);
        prim.vertex3f(off+10.f, 10.f, 20.f );
        prim.vertex3f(off+10.f,  0.f, 20.f );
        prim.end(d);

        prim.color3f(1.f, 0.f, 1.f);
        prim.begin(GL_TRIANGLE_FAN);
        GlDirect_draw::Attr_id id = prim.vertex3f(5.f,  5.f, 10.f  );
        prim.vertex3f(off+0.f,  0.f, 20.f  );
        prim.vertex3f(off+0.f, 10.f, 20.f);
        prim.vertex3f(off+10.f, 10.f, 20.f );
        prim.vertex3f(off+10.f,  0.f, 20.f );
        prim.end(d);

        static float t = 10.f;
        prim.begin_update(GL_TRIANGLE_FAN);
        prim.set(id, GlDirect_draw::ATTR_CURRENTS, off+5.f, 5.f, t);
        prim.end_update();

        prim.color3f(0.f, 1.f, 0.f);
        prim.begin_update();
        prim.set(id, GlDirect_draw::ATTR_CURRENTS, off+5.f, 5.f, t);
        prim.end_update();

        prim.begin_update();
        prim.set(id, GlDirect_draw::ATTR_POSITION, off+5.f, 5.f, t);
        prim.set(id, GlDirect_draw::ATTR_COLOR   , 0.f, 0.f, t/10.f, 1.f);
        prim.end_update();

        t =  t > 10.f ? 0.f : t + 0.2f;

        if(!d) prim.draw();

        prim.clear();

        d = false;
    }
}

#endif
