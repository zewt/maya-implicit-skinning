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
#ifndef ANIMATED_MESH_ENUM_HPP__
#define ANIMATED_MESH_ENUM_HPP__

// =============================================================================
namespace EAnimesh {
// =============================================================================

/// Define some flags corresponding to different cases a vertex is in when
/// fitted into the implicit surfaces with the gradient march.
/// This helps to see what is happening.
enum Vert_state {
    POTENTIAL_PIT,         ///< the vertex is going away from its base potential
    GRADIENT_DIVERGENCE,   ///< collision detected
    NB_ITER_MAX,           ///< stopped before reaching base potential because the surface is too far
    NOT_DISPLACED,         ///< The vertex is already on the right iso-surface
    FITTED,                ///< The vertex has been succesfully fitted on its base potential
    OUT_VERT,              ///< Vertex starts inside another surface than its own
    NORM_GRAD_NULL,        ///< Gradient norm is null can't set a proper marching direction
    // Always keep this at the end ------------------------
    NB_CASES
};

// -----------------------------------------------------------------------------

/// Different Possibilities to compute the vertex color of the animesh
enum Color_type {
    CLUSTER,                  ///< Color different by bone cluster
    NEAREST_JOINT,            ///< Color different by joint cluster
    BASE_POTENTIAL,           ///< Color based on the potential
    GRAD_POTENTIAL,           ///< The current gradient potential of the vertex
    SSD_INTERPOLATION,        ///< Color based on the ssd interpolation factor
    SMOOTHING_WEIGHTS,        ///< Color based on the smoothing weights
    ANIM_SMOOTH_LAPLACIAN,    ///< Color based on the animated smoothing weights
    ANIM_SMOOTH_CONSERVATIVE, ///< Color based on the animated smoothing weights
    NORMAL,                   ///< Color based on mesh current animated normals
    USER_DEFINED,             ///< Uniform color defined by user
    SSD_WEIGHTS,              ///< SSD weights are painted on the mesh
    VERTICES_STATE,           ///< Color based on the stop case encounter while fitting @see EAnimesh::Vert_state
    MVC_SUM                   ///< Color mean value coordinates sum at each vertex
};

// -----------------------------------------------------------------------------

enum Smooth_type {
    LAPLACIAN,     ///< Based on the centroid of the first ring neighborhood
    CONSERVATIVE,  ///< Try to minimize changes with the rest position
    TANGENTIAL,    ///< Laplacian corrected with the mesh normals
    HUMPHREY       ///< Laplacian corrected with original points position
};

// -----------------------------------------------------------------------------

/// Geometric deformations mode of the vertices
enum Blending_type {
    DUAL_QUAT_BLENDING = 0, ///< Vertices are deformed with dual quaternions
    MATRIX_BLENDING         ///< Vertices are deformed with the standard SSD
};

// -----------------------------------------------------------------------------

/// Painting modes for the different mesh attributes
enum Paint_type {
    PT_SSD_INTERPOLATION,
    PT_SSD_WEIGHTS,
    PT_CLUSTER
};

// -----------------------------------------------------------------------------

}
// END EAnimesh NAMESPACE ======================================================

#endif // ANIMATED_MESH_ENUM_HPP__
