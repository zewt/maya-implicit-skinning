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
#ifndef RENDER_CONTEXT_HPP_
#define RENDER_CONTEXT_HPP_

#include "depth_peeling.hpp"
#include "cuda_utils.hpp"

class GlTex2D;
class GlPick_FBO;

/// @class Render_context
/// @brief Holds rendering context (image buffers pbos textures)
/// for a single viewport
class Render_context {
public:
    #ifndef __CUDACC__
    struct float4 { float x,y,z,w; };
    #endif

    Render_context(int w, int h);

    ~Render_context();

    void reshape(int w, int h);
    void allocate(int width, int height);

    // -------------------------------------------------------------------------
    /// @name Getters & setters
    // -------------------------------------------------------------------------

    int width()  const { return _width;  }
    int height() const { return _height; }

    GlBuffer_obj* pbo_color(){ return _pbo_color; }
    GlBuffer_obj* pbo_depth(){ return _pbo_depth; }

    float4* d_render_buff(){ return _d_rendu_buf;       }
    float*  d_depth_buff (){ return _d_rendu_depth_buf; }

    GlTex2D* frame_tex(){ return _frame_tex; }

    Peeler* peeler(){ return _peeler; }

    GlPick_FBO* picker(){ return _picker; }

    // -------------------------------------------------------------------------
    /// @name Rendering states
    // -------------------------------------------------------------------------

    /// deactivate transparency when true
    bool _plain_phong;

    /// enable textures in plain phong
    bool _textures;

    /// Draw the mesh when true
    bool _draw_mesh;

    /// activate raytracing of implicit surface
    bool _raytrace;

    /// Draw skeleton or graph
    bool _skeleton;

    /// Draw the mesh in rest pose
    bool _rest_pose;

private:
    // -------------------------------------------------------------------------
    /// @name Datas
    // -------------------------------------------------------------------------
    int _width, _height;

    GlPick_FBO* _picker; ///< Fbo for picking primitives

    /// The pbo that contains the result of the raytracing done by cuda
    GlBuffer_obj* _pbo_color;
    GlBuffer_obj* _pbo_depth;

    GlTex2D* _frame_tex;

    float4* _d_img_buffer;
    float4* _d_bloom_buffer;
    float4* _d_rendu_buf;
    float*  _d_rendu_depth_buf;

    Peeler* _peeler;
};

#endif // RENDER_CONTEXT_HPP_
