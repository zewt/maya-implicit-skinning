#ifndef MAYA_DATA_H
#define MAYA_DATA_H

#include "loader_mesh.hpp"

#include <maya/MStatus.h>
#include <maya/MObject.h>
#include <maya/MDagPath.h>
#include <maya/MPlug.h>
#include <maya/MMatrix.h>
#include <map>

namespace MayaData
{
    MStatus load_mesh(MObject inputObject, Loader::Abs_mesh &mesh, MMatrix vertexTransform = MMatrix::identity);
    void loadSkeletonHierarchyFromSkinCluster(const std::map<int,MDagPath> &logicalIndexToInfluenceObjects, std::map<int,int> &logicalIndexToParentIdx);
}

#endif
