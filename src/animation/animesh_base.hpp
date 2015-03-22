#ifndef ANIMESH_BASE_HPP
#define ANIMESH_BASE_HPP

#include "animesh_enum.hpp"
#include "skeleton.hpp"
#include "mesh.hpp"

#include <vector>


// This class contains the public interface to Animesh.  This is separated from Animesh
// to allow creating and accessing Animesh without pulling in CUDA includes, to work around
// namespace collisions between Maya and CUDA.  This file can be included in NO_CUDA files,
// but Animesh.hpp can't.
struct Animesh;
class AnimeshBase {
public:
    static AnimeshBase *create(const Mesh *mesh, std::shared_ptr<const Skeleton> skel);
    virtual ~AnimeshBase() { }

    // Get the loaded skeleton.
    virtual const Skeleton *get_skel() const = 0;

    // Get the mesh.
    virtual const Mesh *get_mesh() const = 0;

    /// Computes the potential at each vertex of the mesh. When the mesh is
    /// animated, if implicit skinning is enabled, vertices move so as to match
    /// that value of the potential.  Returns the result, which may then be loaded
    /// with set_base_potential().
    virtual void calculate_base_potential(std::vector<float> &out) const = 0;

    // Read and write the base potential (and gradient).
    virtual void get_base_potential(std::vector<float> &pot) const = 0;
    virtual void set_base_potential(const std::vector<float> &pot) = 0;

    /// Transform the vertices of the mesh given the rotation at each bone.
    /// Transformation is computed from the initial position of the mesh
    /// @param type specify the technic used to compute vertices deformations
    virtual void transform_vertices() = 0;

    // Return the number of vertices in the mesh.  Calls to copy_vertices must have the
    // same number of vertices.
    virtual int get_nb_vertices() const = 0;

    // Read the vertices.
    virtual void get_vertices(std::vector<Point_cu>& anim_vert) const = 0;

    // Copy the given vertices into the mesh.
    virtual void set_vertices(const std::vector<Vec3_cu> &vertices) = 0;

    virtual inline void set_smooth_factor(int i, float val) = 0;

    virtual void set_nb_transform_steps(int nb_iter) = 0;
    virtual void set_final_fitting(bool value) = 0;
    virtual void set_smoothing_weights_diffusion_iter(int nb_iter) = 0;
    virtual void set_smoothing_iter (int nb_iter ) = 0;
    virtual void set_smooth_mesh    (bool state  ) = 0;
    virtual void set_local_smoothing(bool state  ) = 0;
    virtual void set_smooth_force_a (float alpha ) = 0;
    virtual void set_smooth_force_b (float beta  ) = 0;
    virtual void set_smoothing_type (EAnimesh::Smooth_type type ) = 0;
};

#endif
