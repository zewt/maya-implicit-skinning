#ifndef MAYA_HELPERS_H
#define MAYA_HELPERS_H

#if !defined(NO_CUDA)
#error This header requires NO_CUDA.
#endif

#include <maya/MArrayDataHandle.h>
#include <maya/MDataHandle.h>
#include <maya/MObject.h>
#include <maya/MPlug.h>
#include <maya/MString.h>
#include <maya/MMatrix.h>
#include <maya/MDagPath.h>
#include <string>
#include <vector>
#include <map>
#include <set>
using namespace std;

#include "transfo.hpp"

namespace DagHelpers
{
    // This exists only to work around Maya's float3, etc. vector types.  They're array typedefs,
    // which is inconsistent with the other types in that it can't be returned by value.
    struct simpleFloat3 {
        simpleFloat3(): x(0), y(0), z(0) { }
        simpleFloat3(float3 value): x(value[0]), y(value[1]), z(value[2]) { }
        void to_float3(float3 &value) { value[0] = x; value[1] = y; value[2] = z; }

        float x, y, z;
    };
    Transfo MMatrixToTransfo(const MMatrix &mmat);

    MStatus getConnectedPlugWithName(MObject inputNode, std::string name, MObject &result);
    MMatrix getMatrixFromPlug(MPlug plug, MStatus *status);
    MStatus findAncestorDeformer(MObject node, MFn::Type type, MObject &resultNode);
    MStatus getMDagPathsFromSkinCluster(MObject skinClusterNode, std::vector<MDagPath> &out);
    int findClosestAncestor(const std::vector<MDagPath> &dagPaths, MDagPath dagPath);
    MStatus getInputGeometryForSkinClusterPlug(MObject skinClusterNode, MPlug &outPlug);
    MStatus setMatrixPlug(MObject node, MObject attr, MMatrix matrix);
    MStatus getMatrixPlug(MObject node, MObject attr, MMatrix &matrix);
    MStatus setPlug(MPlug &plug, MMatrix value);

    template<class T>
    T readHandle(MDataHandle handle, MStatus *status);

    template<> inline MDataHandle readHandle(MDataHandle handle, MStatus *status) { return handle; }
    template<> inline int readHandle(MDataHandle handle, MStatus *status) { return handle.asInt(); }
    template<> inline float readHandle(MDataHandle handle, MStatus *status) { return handle.asFloat(); }
    template<> inline MMatrix readHandle(MDataHandle handle, MStatus *status) { return handle.asMatrix(); }
    template<> inline bool readHandle(MDataHandle handle, MStatus *status) { return handle.asBool(); }
    template<> inline simpleFloat3 readHandle(MDataHandle handle, MStatus *status) { return (simpleFloat3) handle.asFloat3(); }

    template<class T>
    inline T readHandle(MDataBlock &dataBlock, const MObject &attribute, MStatus *status)
    {
        MDataHandle itemHandle = dataBlock.inputValue(attribute, status);
        if(*status != MS::kSuccess) return T();
        return readHandle<T>(itemHandle, status);
    }

    template<class T>
    inline T readArrayHandle(MArrayDataHandle arrayHandle, MStatus *status)
    {
        MDataHandle itemHandle = arrayHandle.inputValue(status);
        if(*status != MS::kSuccess) return T();
        return readHandle<T>(itemHandle, status);
    }

    template<class T>
    inline T readArrayHandleLogicalIndex(MArrayDataHandle arrayHandle, int logicalIndex, MStatus *status)
    {
        *status = arrayHandle.jumpToElement(logicalIndex);
        if(*status != MS::kSuccess) return T();

        return readArrayHandle<T>(arrayHandle, status);
    }

    bool getPlugConnectedTo(const MObject& node, const MString &attribute, MPlug& connectedPlug);
    MObject getNodeConnectedTo(const MObject& node, const MString &attribute);
    MObject getSourceNodeConnectedTo ( const MObject& node, const MString& attribute );
    MObject getSourceNodeConnectedTo ( const MPlug& inPlug );
    bool getPlugConnectedTo ( const MObject& node, const MString &attribute, MPlug& connectedPlug );
    bool getPlugConnectedTo ( const MPlug& inPlug, MPlug& plug );
    MObject getNodeConnectedTo ( const MPlug& plug );


    MStatus setPlugValue(MPlug& plug, float x, float y);
    MStatus setPlugValue(MPlug& plug, float x, float y, float z);
    MStatus getPlugValue(const MPlug& plug, float& x, float& y, float& z);

    /*
     * This class works around strange behavior in Maya's dependency system.  If A depends on B,
     * and B depends on C, then by definition A depends on C.  However, Maya's dependencies don't
     * handle this correctly, requiring the user to declare every recursive dependency explicitly.
     * That's unmaintainable with more than a couple dependencies, so this class handles it.
     */
    class MayaDependencies
    {
    public:
        // Changes to from affect to, and everything that to effects.
        void add(const MObject &from, const MObject &to);

        // Apply all dependencies.
        MStatus apply();

    private:
        MStatus apply_one(const MObject *from, const MObject *to, set<const MObject*> &stack);

        // MObject isn't ordered, so we can't use it in a map normally.  Since we're only using this with static attribute
        // objects, we can make sure that we only ever have a single instance of the MObject, as long as we're careful to
        // not make copies.
        map<const MObject*, set<const MObject*> > dependencies;
    };
}

#endif
