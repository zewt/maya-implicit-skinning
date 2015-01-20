#ifndef MAYA_DATA_H
#define MAYA_DATA_H

#include "loader_mesh.hpp"
#include "loader_skel.hpp"

#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MMatrix.h>
#include <vector>

namespace MayaData
{
    MStatus load_mesh(MObject inputObject, Loader::Abs_mesh &mesh, MMatrix vertexTransform = MMatrix::identity);
    MStatus loadSkeletonHierarchyFromSkinCluster(MObject skinClusterNode, std::vector<int> &parentIndexes);
}

#endif
