#ifndef LOADER_MESH_HPP__
#define LOADER_MESH_HPP__

#include <vector>

#include "point_cu.hpp"

namespace Loader {

struct Tri_face {
    unsigned v[3]; ///< vertex indices for the triangle
    int n[3];      ///< normal indices for the triangle

    /// ctor
    Tri_face() {
        v[0] = v[1] = v[2] =  0;
        // -1 indicates not used
        n[0] = n[1] = n[2] = -1;
    }
};


/**
  @struct Abs_mesh
  @brief An abstract mest only represent triangle meshes.
  An abstract mesh contains the triangulated representation of a mesh
  in '_triangles' but also it's version with quads to enable nicer rendering
  in wireframe mode (attribute _render_faces)
*/
struct Abs_mesh {
    std::vector<Point_cu>    _vertices;
    std::vector<Vec3_cu>    _normals;
    std::vector<Tri_face>  _triangles;  ///< the triangulated faces
};

}

#endif
