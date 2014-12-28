#ifndef MAYA_HELPERS_H
#define MAYA_HELPERS_H

#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MMatrix.h>
#include <maya/MDagPath.h>
#include <string>
#include <vector>

#include "cpu_transfo.hpp"

namespace DagHelpers
{
    Loader::CpuTransfo MMatrixToCpuTransfo(const MMatrix &dagMat);

    MStatus getConnectedPlugWithName(MPlug inputPlug, std::string name, MPlug &result);
    MMatrix getMatrixFromPlug(MPlug plug, MStatus *status);
    MStatus findAncestorDeformer(MPlug inputPlug, MFn::Type type, MPlug &resultPlug);
    MStatus getMDagPathsFromSkinCluster(MPlug skinClusterPlug, std::vector<MDagPath> &out);
    int findClosestAncestor(const std::vector<MDagPath> &dagPaths, MDagPath dagPath);
    MStatus getInputGeometryForSkinClusterPlug(MPlug skinClusterPlug, MObject &plug);
    MStatus setMatrixPlug(MPlug plug, MObject attr, MMatrix matrix);

    bool getPlugConnectedTo(const MObject& node, const MString &attribute, MPlug& connectedPlug);
    MObject getNodeConnectedTo(const MObject& node, const MString &attribute);
    MObject getSourceNodeConnectedTo ( const MObject& node, const MString& attribute );
    MObject getSourceNodeConnectedTo ( const MPlug& inPlug );
    bool getPlugConnectedTo ( const MObject& node, const MString &attribute, MPlug& connectedPlug );
    bool getPlugConnectedTo ( const MPlug& inPlug, MPlug& plug );
    MObject getNodeConnectedTo ( const MPlug& plug );
}

#endif
