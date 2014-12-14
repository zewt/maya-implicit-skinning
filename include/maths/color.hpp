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
#ifndef COLOR_HPP__
#define COLOR_HPP__

#include "vec3_cu.hpp"
#include "cuda_compiler_interop.hpp"
#include "port_glew.h"
/**
  @name Color
  @brief storing and handling RBGA colors

  This aims to factorize what's related to color with predefined colors
*/
struct Color{

    IF_CUDA_DEVICE_HOST
    Color(){
        r = g = b = 0.f;
    }

    /// Sets rgb channels to the value 'grey' and channel a = 1.f
    IF_CUDA_DEVICE_HOST Color(float grey){ r = g = b = grey; a = 1.f; }

    IF_CUDA_DEVICE_HOST
    Color(float r_, float g_, float b_, float a_ = 1.f){
        r = r_; g = g_; b = b_; a = a_;
    }

    // -------------------------------------------------------------------------
    /// @name Color generation
    // -------------------------------------------------------------------------

    /// Return a pseudo random color. the color is the same for each 'i' but
    /// supposedly very different between i and i+1
    IF_CUDA_DEVICE_HOST
    static Color pseudo_rand(int i){
        Color c;
        c.r = ((i * 212435 + 2312  ) % 255) * 1.f/255.f;
        c.g = ((i * 878923 + 123342) % 255) * 1.f/255.f;
        c.b = ((i * 18464  + 79874 ) % 255) * 1.f/255.f;
        c.a = ((i * 563956 + 951841) % 255) * 1.f/255.f;
        return c;
    }

    /// Generate from the parameter w [0 1] a smooth color from red to blue:
    /// 0    -> blue(0,1,0)
    /// 0.25 -> pale blue(0,1,1)
    /// 0.5  -> green(0,1,0)
    /// 0.75 -> yellow(1,1,0)
    /// 1    -> red(1,0,0)
    IF_CUDA_DEVICE_HOST
    static Color heat_color(float w)
    {
        Color c(0.5f, 0.f, 1.f, 1.f);
        if(w < 0.25f){
            w *= 4.f;
            c.set(0.f, w, 1.f, 1.f);
        }else if(w < 0.5f){
            w = (w-0.25f) * 4.f;
            c.set(0.f, 1.f, 1.f-w, 1.f);
        }else if(w < 0.75f){
            w = (w-0.5f) * 4.f;
            c.set(w, 1.f, 0.f, 1.f);
        }else{
            w = (w-0.75f) * 4.f;
            c.set(1.f, 1.f-w, 0.f, 1.f);
        }
        return c;
    }

    // -------------------------------------------------------------------------
    /// @name Basic preset color
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST static Color black()  { return Color(0.f, 0.f, 0.f); }
    IF_CUDA_DEVICE_HOST static Color blue()   { return Color(0.f, 0.f, 1.f); }
    IF_CUDA_DEVICE_HOST static Color green()  { return Color(0.f, 1.f, 0.f); }
    IF_CUDA_DEVICE_HOST static Color cyan()   { return Color(0.f, 1.f, 1.f); }
    IF_CUDA_DEVICE_HOST static Color red()    { return Color(1.f, 0.f, 0.f); }
    IF_CUDA_DEVICE_HOST static Color purple() { return Color(1.f, 0.f, 1.f); }
    IF_CUDA_DEVICE_HOST static Color yellow() { return Color(1.f, 1.f, 0.f); }
    IF_CUDA_DEVICE_HOST static Color white()  { return Color(1.f, 1.f, 1.f); }

    // -------------------------------------------------------------------------
    /// @name Getters & Setters
    // -------------------------------------------------------------------------

    /// Convert to Vec3_cu alpha channel is dropped
    IF_CUDA_DEVICE_HOST
    Vec3_cu to_vec3() const { return Vec3_cu(r, g, b); }

    IF_CUDA_DEVICE_HOST
    void set(float r_, float g_, float b_, float a_){
        r = r_; g = g_; b = b_; a = a_;
    }

    void set_gl_state() const {
        glColor4f(r, g, b, a);
    }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

    float r, g, b, a;
};

#endif// COLOR_HPP__
