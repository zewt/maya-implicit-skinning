#ifndef IMPLICIT_SKIN_DEFORMER_H
#define IMPLICIT_SKIN_DEFORMER_H

#ifndef NO_CUDA
#error This header requires NO_CUDA.
#endif

#include "mesh.hpp"
#include "animated_mesh_ctrl.hpp"
#include "maya_helpers.hpp"
#include "sample_set.hpp"

#include <maya/MPxDeformerNode.h> 
#include <maya/MPxSurfaceShape.h>
#include <maya/MPxSurfaceShapeUI.h>
#include <maya/MPxComponentShape.h>
#include <maya/MPxGeometryOverride.h>
#include <maya/MGeometry.h>

#include <memory>

#include "marching_cubes/marching_cubes.hpp"
#include "implicit_surface_geometry_override.hpp"
#include "implicit_surface.hpp"
#include "implicit_blend.hpp"
#include "implicit_deformer.hpp"
#include "implicit_surface_data.hpp"

#endif
