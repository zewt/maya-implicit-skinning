#ifndef MARCHING_CUBES_H
#define MARCHING_CUBES_H

#include <maya/MObject.h>

#include "mesh_data.h"
#include "bone.hpp"

namespace MarchingCubes
{
    // Compute the geometry to preview the given bone, and append it to meshGeom.
    void compute_surface(MeshGeom &geom, const Bone_hrbf *bone);
    
    // Return a MFnMeshData node for MeshGeom.
    MObject create_visualization_geom(const MeshGeom &srcGeom, MStatus *status);
}

#endif
