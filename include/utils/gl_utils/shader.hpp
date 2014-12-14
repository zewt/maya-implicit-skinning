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
#ifndef SHADER_HPP__
#define SHADER_HPP__

#include <string>
#include <map>
#include "port_glew.h"

#ifndef GL_GEOMETRY_SHADER
#define GL_GEOMETRY_SHADER (GL_GEOMETRY_SHADER_EXT)
#endif

/** @class Shader

*/
// =============================================================================
class Shader{
// =============================================================================
public:
    /// basic constructor
    /// @param shaderType either GL_VERTEX_SHADER, GL_GEOMETRY_SHADER or
    /// GL_FRAGMENT_SHADER
    Shader(GLuint shaderType);

    /// compile shader from a text source file
    Shader(const std::string& source_path, GLuint shaderType);

    /// adds a macro definition to the shader programm. If the symbol already
    /// exists it will be overwriten by the last entry
    /// @warning must be done before calling load_source otherwise it won't
    /// be effective.
    void add_define(const std::string& name, const std::string& value);

    /// load and compile source file from a text file
    int load_file(const std::string& source_path) const;

    /// load and compile source file from a raw string
    int load_source(const char* source) const;

    /// get compilation status
    int get_status() const;

    /// Check the current status and print error message if needed
    int check_status(const std::string& name = "") const;

    /// get shader's id
    GLuint get_type() const { return _type; }
    operator GLuint() const;
    ~Shader();
private:
    GLuint _id;
    GLuint _type;
    typedef std::map<std::string, std::string> Macro_list;
    Macro_list _symbols;
};
// =============================================================================


// =============================================================================
struct Shader_prog{
// =============================================================================
public:

    /// basic constructor
    Shader_prog();

    /// creation from 2 shaders (compiled)
    Shader_prog(const Shader& vs, const Shader& fs);

    ~Shader_prog();

    /// set both vertex & fragment shaders (compiled)
    /// @return true if all went well
    int set_shaders(const Shader& vs, const Shader& fs);

    /// set any compiled shader
    /// @return true if all went well
    int set_shader(const Shader& vs);

    /// get linkage status
    int get_status() const;

    /// link the 2 shaders
    int link() const;

    /// use the program
    int use() const;

    /// do not use the program
    static int unuse();

    /// @return programm id currently used or 0
    static int currently_used();

    /// maximum number of attributes per vertex
    static int get_max_attributes();

    // -------------------------------------------------------------------------
    /// @name set a uniform variable
    // -------------------------------------------------------------------------
    /// @{
    //int set_uniform(const char* name, unsigned v0) const;

    int set_uniform(const char* name, int v0) const;
    int set_uniform(const char* name, int v0, int v1) const;
    int set_uniform(const char* name, int v0, int v1, int v2) const;
    int set_uniform(const char* name, int v0, int v1, int v2, int v3) const;
    int set_uniform(const char* name, GLsizei count, int* values) const;

    int set_uniform(const char* name, float v0) const;
    int set_uniform(const char* name, float v0, float v1) const;
    int set_uniform(const char* name, float v0, float v1, float v2) const;
    int set_uniform(const char* name, float v0, float v1, float v2, float v3) const;
    int set_uniform(const char* name, GLsizei count, float* values) const;

    int set_tex_unit(const char* name, int idx) const { return set_uniform(name, idx); }

    /// Sets a uniform Matrix4x4
    /// @param is_row_major : does the matrices are row major. (by default
    /// opengl matrices are column major)
    int set_mat4x4(const char* name, const float* values, GLsizei count = 1, bool is_row_major = false ) const;
    /// @}

    /// Bind a vertex attribute to a specific index.
    /// @warning this will take effects after the next linking with link().
    /// Usually when linking an index is automatically associated to the
    /// attributes names in the vertex shader. There is no reason to use this
    /// method unless you specify by hand every attributes included the built-in
    /// gl_Position, gl_Normal, glColor and co.
    void bind_attribute(const char* name, int index_attr);

    /// @return index of the attribute named 'name'. If -1 is returned it means
    /// that the atribute is not used by the shaders.
    int get_attribute_location(const char* name);

    int get_id() const { return _id; }

private:
    GLuint _id;
    GLuint _vs_id;
    GLuint _gs_id;
    GLuint _fs_id;
};
// =============================================================================

#endif // SHADER_HPP__
