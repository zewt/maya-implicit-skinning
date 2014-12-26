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
#ifndef SCENE_ENUM_HPP
#define SCENE_ENUM_HPP

enum Scene_Type{
    NONE = 0,
    SKELETON,
    COMPOSITION_TREE,

    MIN_T,
    MAX_T,
    SUM_T,
    RICCI_T,
    SOFT_DIFF_T,
    SHARP_DIFF_T,
    CANI_CONTACT_T,
    Wyvill_GRAPH_OPERATOR_T,
    RESTRICTED_BLENDING_T,
    CIRCLE_LINES,
    CIRCLE_HYPERBOLA_OPEN,
    CIRCLE_HYPERBOLA_CLOSED_D,
    CIRCLE_HYPERBOLA_CLOSED_H,
    CIRCLE_HYPERBOLA_CLOSED_T,
    ULTIMATE_HYPERBOLA_OPEN,
    ULTIMATE_HYPERBOLA_CLOSED_H,
    ULTIMATE_HYPERBOLA_CLOSED_T,
    BULGE_HYPERBOLA_OPEN,
    BULGE_HYPERBOLA_CLOSED_H,
    BULGE_HYPERBOLA_CLOSED_T,
    CIRCLE_INTERSECTION_HYPERBOLA_OPEN,
    CIRCLE_INTERSECTION_HYPERBOLA_CLOSED_H,
    CIRCLE_INTERSECTION_HYPERBOLA_CLOSED_T,
    ULTIMATE_INTERSECTION_HYPERBOLA_OPEN,
    ULTIMATE_INTERSECTION_HYPERBOLA_CLOSED_H,
    ULTIMATE_INTERSECTION_HYPERBOLA_CLOSED_T,
    CIRCLE_DIFF_HYPERBOLA_OPEN,
    CIRCLE_DIFF_HYPERBOLA_CLOSED_H,
    CIRCLE_DIFF_HYPERBOLA_CLOSED_T,
    ULTIMATE_DIFF_HYPERBOLA_OPEN,
    ULTIMATE_DIFF_HYPERBOLA_CLOSED_H,
    ULTIMATE_DIFF_HYPERBOLA_CLOSED_T
};

#endif // SCENE_ENUM_HPP
