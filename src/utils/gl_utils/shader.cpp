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
#include "shader.hpp"

#include <iostream>
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include "glassert.h"

// Private tools ===============================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static char* text_file_read(const char* fn)
{
    FILE* fp;
    char* content = NULL;
    int count = 0;
    if (fn == NULL) return 0;

    fp = fopen(fn,"rt");
    if(fp == NULL) return 0;

    fseek(fp, 0, SEEK_END);
    count = ftell(fp);
    rewind(fp);

    if (count > 0)
    {
        content = (char *)malloc(sizeof(char) * (count+1));
        count   = fread(content,sizeof(char),count,fp);
        content[count] = '\0';
    }

    fclose(fp);

    return content;
}

// End private tools ===========================================================

// SHADER CLASS IMPLEMENTATION =================================================

Shader::Shader(GLuint shaderType)
{
    _id   = glCreateShader(shaderType);
    _type = shaderType;
    GL_CHECK_ERRORS();
}

// -----------------------------------------------------------------------------

Shader::Shader(const std::string& source_name, GLuint shaderType)
{
    _id   = glCreateShader(shaderType);
    _type = shaderType;
    GL_CHECK_ERRORS();
    load_file(source_name);
}

// -----------------------------------------------------------------------------

void Shader::add_define(const std::string& name, const std::string& value)
{
    _symbols[name] = value;
}

// -----------------------------------------------------------------------------

int Shader::load_file(const std::string& source_name) const
{
    char* source  = text_file_read(source_name.c_str());
    if(source == 0)
    {
        std::cerr << "error : could not read shader file :" << source_name << std::endl;
        return 0;
    }
    int s = load_source( source );
    free(source);

    return s;
}

// -----------------------------------------------------------------------------

int Shader::load_source(const char* source) const
{
    std::string defines;

    for(Macro_list::const_iterator it = _symbols.begin(); it != _symbols.end(); ++it)
        defines += "#define " + it->first + " " + it->second + "\n";

    const char* file[2] = {defines.c_str(), (const char*)source};
    glAssert( glShaderSource(_id, 2, file, 0) );
    glAssert( glCompileShader(_id) );

    return check_status();
}

// -----------------------------------------------------------------------------

int Shader::check_status(const std::string& name) const
{
    int len;
    if(get_status() != GL_TRUE)
    {
        std::cerr << "error : shader compilation :" << name << std::endl;
        const int maxloglen = 4096;
        char log[maxloglen];
        glAssert( glGetShaderInfoLog(_id, maxloglen, &len, log) );
        std::cerr << log << std::endl;
        fflush(stderr);
        return 0;
    }
    return 1;
}

// -----------------------------------------------------------------------------


int Shader::get_status() const
{
    int status;
    glAssert( glGetShaderiv(_id, GL_COMPILE_STATUS, &status) );
    return status;
}

// -----------------------------------------------------------------------------

Shader::operator GLuint() const{ return _id; }

// -----------------------------------------------------------------------------

Shader::~Shader(){ glAssert( glDeleteShader(_id) ); }

// END SHADER CLASS IMPLEMENTATION =============================================




// SHADER_PROGRAMM CLASS IMPLEMENTATION ========================================

Shader_prog::Shader_prog() :
    _id(glCreateProgram()),
    _vs_id(0),
    _gs_id(0),
    _fs_id(0)
{  }

// -----------------------------------------------------------------------------

Shader_prog::Shader_prog(const Shader& vs, const Shader& fs):
    _id(glCreateProgram()),
    _gs_id(0)
{
    if(vs.get_type() != GL_VERTEX_SHADER){
        std::cerr << "Expected a vertex shader !" << std::endl;
        assert(vs.get_type() == GL_VERTEX_SHADER);
    }

    if(fs.get_type() != GL_FRAGMENT_SHADER){
        std::cerr << "Expected a fragment shader !" << std::endl;
        assert(fs.get_type() == GL_FRAGMENT_SHADER);
    }

    set_shader(vs);
    set_shader(fs);
}

// -----------------------------------------------------------------------------

int Shader_prog::set_shaders(const Shader& vs, const Shader& fs)
{
    return set_shader(vs) & set_shader(fs);
}

// -----------------------------------------------------------------------------

int Shader_prog::set_shader(const Shader& sh)
{
    if(sh.get_status() == GL_TRUE)
    {
        switch( sh.get_type() )
        {
        case GL_VERTEX_SHADER:   _vs_id = sh;break;
        case GL_GEOMETRY_SHADER: _gs_id = sh;break;
        case GL_FRAGMENT_SHADER: _fs_id = sh;break;
        };
        glAssert( glAttachShader(_id, sh) );
        return 1;
    }

    switch( sh.get_type() )
    {
    case GL_VERTEX_SHADER:   _vs_id = 0;break;
    case GL_GEOMETRY_SHADER: _gs_id = 0;break;
    case GL_FRAGMENT_SHADER: _fs_id = 0;break;
    };

    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::get_status() const
{
    int status;
    glAssert( glGetProgramiv(_id, GL_LINK_STATUS, &status) );
    return status;
}

// -----------------------------------------------------------------------------

int Shader_prog::link() const
{
    if(_vs_id != 0 || _fs_id != 0)
    {
        glAssert( glLinkProgram(_id) );
        if(get_status() != GL_TRUE)
        {
            int len;
            fprintf (stderr, "error : linking program \n");
            const int maxloglen=4096;
            char log[maxloglen];
            glAssert( glGetProgramInfoLog(_id, maxloglen, &len, log) );
            fprintf (stderr, "%s\n", log);
            fflush(stderr);
            return 0;
        }
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::use() const
{
    if(get_status() == GL_TRUE)
    {
        glAssert( glUseProgram(_id) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::unuse()
{
    glAssert( glUseProgram(0) );
    return 1;
}

// -----------------------------------------------------------------------------

int Shader_prog::currently_used()
{
    int id;
    glAssert( glGetIntegerv(GL_CURRENT_PROGRAM, &id) );
    return id;
}

// -----------------------------------------------------------------------------

int Shader_prog::get_max_attributes()
{
    int n;
    glAssert( glGetIntegerv(GL_MAX_VERTEX_ATTRIBS, &n) );
    return n;
}

// -----------------------------------------------------------------------------
/*
int Shader_prog::set_uniform(const char* name, unsigned v0) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform1ui(res, v0) );
        return 1;
    }
    return 0;
}
*/
// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, int v0) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform1i(res, v0) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, int v0, int v1) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform2i(res, v0, v1) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, int v0, int v1, int v2) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform3i(res, v0, v1, v2) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, int v0, int v1, int v2, int v3) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform4i(res, v0, v1, v2, v3) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, GLsizei count, int* values) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform1iv(res, count, values) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, float v0) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform1f(res, v0) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, float v0, float v1) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform2f(res, v0, v1) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, float v0, float v1, float v2) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform3f(res, v0, v1, v2) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, float v0, float v1, float v2, float v3) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform4f(res, v0, v1, v2, v3) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_uniform(const char* name, GLsizei count, float* values) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniform1fv(res, count, values) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

int Shader_prog::set_mat4x4(const char* name, const float *values, GLsizei count, bool is_row_major) const
{
    assert(Shader_prog::currently_used() == (int)_id);
    int res = glGetUniformLocation(_id, name);
    GL_CHECK_ERRORS();
    if(res != -1)
    {
        glAssert( glUniformMatrix4fv(res, count, is_row_major, values) );
        return 1;
    }
    return 0;
}

// -----------------------------------------------------------------------------

void Shader_prog::bind_attribute(const char* name, int index_attr)
{
    glAssert( glBindAttribLocation( _id, index_attr, name) );
}

// -----------------------------------------------------------------------------

int Shader_prog::get_attribute_location(const char* name)
{
    int attr_idx = -1;
    attr_idx = glGetAttribLocation(_id, name);
    GL_CHECK_ERRORS();
    return attr_idx;
}

// -----------------------------------------------------------------------------

Shader_prog::~Shader_prog()
{
    if(_vs_id != 0) glAssert( glDetachShader(_id, _vs_id) );
    if(_fs_id != 0) glAssert( glDetachShader(_id, _fs_id) );

    glAssert( glDeleteProgram(_id) );
}

// END SHADER_PROGRAMM CLASS IMPLEMENTATION ====================================
