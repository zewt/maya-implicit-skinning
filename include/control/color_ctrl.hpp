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
#ifndef COLOR_CTRL_HPP__
#define COLOR_CTRL_HPP__

#include "color.hpp"

/** @class Color_ctrl
  @brief Stores the colors used in the viewport to display the mesh, the points etc.

  In order to add a color one have to add a field to the enum list.
  Method set() might be changed if the new color necessit a more complex
  operation than just changing the array value corresponding to the color.
  For instance The color for a mesh necessit to update the GPU memory.
*/
class Color_ctrl {
public:

    Color_ctrl(){
        _list[HRBF_POINTS].set(0.2f, 0.4f, 0.5f, 1.f);
        _list[HRBF_SELECTED_POINTS].set(1.f, 1.f, 0.f, 1.f);
        _list[MESH_DEFECTS].set(1.f, 1.f, 0.f, 1.f);
        _list[MESH_POINTS].set(1.0f, 1.0f, 1.0f, 1.0f);
        _list[MESH_SELECTED_POINTS].set(1.f, 1.f, 0.f, 1.f);
        _list[MESH_OUTLINE].set(1.f, 1.f, 1.f, 1.f);
        _list[MESH_OUTLINE_ACTIVE].set(1.f, 0.f, 0.f, 1.f);
        _list[BACKGROUND].set(0.3f, 0.3f, 0.3f, 1.f);
        _list[VIEWPORTS_MSGE].set(0.0f, 0.0f, 0.0f, 1.f);
        _list[BOUNDING_BOX].set(0.0f, 0.0f, 0.0f, 1.f);
        _list[POTENTIAL_IN].set(0.9f, 0.2f, 0.1f, 1.f);
        _list[POTENTIAL_OUT].set(0.1f, 0.2f, 0.9f, 1.f);
        _list[POTENTIAL_NEG].set(0.9f, 0.9f, 0.2f, 1.f);
        _list[POTENTIAL_POS].set(1.0f, 0.75f, 0.5f, 1.f);
        load_class_from_file();
    }

    enum {
        HRBF_POINTS = 0,       ///< hermite rbf samples color
        HRBF_SELECTED_POINTS,  ///< hermite rbf samples color when selected
        MESH_DEFECTS,          ///< mesh's sides and none manifold verts
        MESH_POINTS,           ///< mesh's points color
        MESH_SELECTED_POINTS,  ///< mesh's points color when selected
        MESH_OUTLINE,          ///< mesh's outline color when selected
        MESH_OUTLINE_ACTIVE,   ///< mesh's outline color when selected and active
        BACKGROUND,            ///< Viewports background
        VIEWPORTS_MSGE,        ///< Textual message color on viewports
        BOUNDING_BOX,
        POTENTIAL_IN,          ///< Viewport background
        POTENTIAL_OUT,         ///< Viewport background
        POTENTIAL_NEG,         ///< Viewport background
        POTENTIAL_POS,         ///< Viewport background
        //----------------------------------------------------------------------
        NB_COLOR               ///< Keep this in the end of the enum
        //----------------------------------------------------------------------
    };

    /// Get a color associated to the enum_field
    Color get(int enum_field);
    /// Set a color associated to the enum_field
    void  set(int enum_field, const Color& cl);

private:
    void save_class_to_file();
    void load_class_from_file();

    Color _list[NB_COLOR];
};

#endif // COLOR_CTRL_HPP__
