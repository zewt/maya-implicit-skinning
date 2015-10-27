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
    NONE,
    LAPLACIAN,     ///< Based on the centroid of the first ring neighborhood
    CONSERVATIVE,  ///< Try to minimize changes with the rest position
    TANGENTIAL,    ///< Laplacian corrected with the mesh normals
    HUMPHREY       ///< Laplacian corrected with original points position
};

}
// END EAnimesh NAMESPACE ======================================================

#endif // ANIMATED_MESH_ENUM_HPP__
