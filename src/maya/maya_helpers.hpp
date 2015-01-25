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
#include <maya/MFnDependencyNode.h>
#include <maya/MMatrix.h>
#include <maya/MDagPath.h>
#include <string>
#include <vector>
#include <map>
#include <set>
using namespace std;

#include "vec3_cu.hpp"
#include "transfo.hpp"
class MFnSkinCluster;

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
    MStatus getWeightsForAllVertices(MObject skinClusterNode, vector<vector<double> > &weightsPerIndex);
    MStatus getMDagPathsFromSkinCluster(MObject skinClusterNode, std::vector<MDagPath> &out);
    int findClosestAncestor(const std::map<int,MDagPath> &logicalIndexToInfluenceObjects, MDagPath dagPath);
    MStatus setMatrixPlug(MObject node, MObject attr, MMatrix matrix);
    MStatus getMatrixPlug(MObject node, MObject attr, MMatrix &matrix);
    MStatus setPlug(MPlug &plug, MMatrix value);
    MObject getShapeUnderNode(MObject node, MStatus *status);
    MStatus lockTransforms(MObject node);
    MStatus setParent(MObject parent, MObject child);
    MObject createTransform(MString name, MStatus status);

    template<class T>
    T readHandle(MDataHandle handle, MStatus *status);

    template<> inline MDataHandle readHandle(MDataHandle handle, MStatus *status) { return handle; }
    template<> inline int readHandle(MDataHandle handle, MStatus *status) { return handle.asInt(); }
    template<> inline short readHandle(MDataHandle handle, MStatus *status) { return handle.asShort(); }
    template<> inline float readHandle(MDataHandle handle, MStatus *status) { return handle.asFloat(); }
    template<> inline MMatrix readHandle(MDataHandle handle, MStatus *status) { return handle.asMatrix(); }
    template<> inline bool readHandle(MDataHandle handle, MStatus *status) { return handle.asBool(); }
    template<> inline simpleFloat3 readHandle(MDataHandle handle, MStatus *status) { return (simpleFloat3) handle.asFloat3(); }
    template<> inline Vec3_cu readHandle(MDataHandle handle, MStatus *status) { const float3 &f = handle.asFloat3(); return Vec3_cu(f[0], f[1], f[2]); }

    template<class T>
    void setHandle(MDataHandle handle, T value);
    
    template<> inline void setHandle(MDataHandle handle, float value) { handle.setFloat(value); }
    template<> inline void setHandle(MDataHandle handle, Vec3_cu value) { handle.set3Float(value.x, value.y, value.z); }

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

    // Read all elements of the given array.
    template<typename T>
    MStatus readArray(MArrayDataHandle arrayHandle, vector<T> &out)
    {
        MStatus status = MStatus::kSuccess;
        out.resize(arrayHandle.elementCount());

        for(int i = 0; i < (int) arrayHandle.elementCount(); ++i)
        {
            status = arrayHandle.jumpToElement(i);
            if(status != MS::kSuccess) return status;
        
            MDataHandle item = arrayHandle.inputValue(&status);
            out[i] = DagHelpers::readHandle<T>(item, &status);
        }
        return MStatus::kSuccess;
    }

    // Set all values of the given array to the contents of data.  The attribute must
    // have setUsesArrayDataBuilder enabled.
    template<typename T>
    MStatus setArray(MDataBlock &dataBlock, MObject attr, const vector<T> &data)
    {
        MStatus status = MStatus::kSuccess;

        MArrayDataBuilder builder(&dataBlock, attr, (int) data.size(), &status);
        if(status != MS::kSuccess) return status;

        for(int i = 0; i < (int) data.size(); ++i)
        {
            MDataHandle valueHandle = builder.addLast(&status);
            if(status != MS::kSuccess) return status;
            DagHelpers::setHandle(valueHandle, data[i]);
        }

        MArrayDataHandle arrayHandle = dataBlock.inputArrayValue(attr, &status);
        if(status != MS::kSuccess) return status;

        status = arrayHandle.set(builder);
        if(status != MS::kSuccess) return status;

        status = arrayHandle.setAllClean();
        if(status != MS::kSuccess) return status;

        return MStatus::kSuccess;
    }

    // Add value to array at the given index.  The attribute must have setUsesArrayDataBuilder
    // enabled.
    template<typename T>
    MStatus addObjectToArray(MDataBlock &dataBlock, MObject attr, int logicalIndex, T value)
    {
        MStatus status = MStatus::kSuccess;
        MArrayDataHandle worldImplicitHandle = dataBlock.outputArrayValue(attr, &status);
        if(status != MS::kSuccess) return status;

        status = worldImplicitHandle.jumpToElement(logicalIndex);
        if(status != MS::kSuccess) return status;

        MArrayDataBuilder builder = worldImplicitHandle.builder(&status);
        if(status != MS::kSuccess) return status;

        MDataHandle outHandle = builder.addElement(logicalIndex, &status);
        if(status != MS::kSuccess) return status;

        status = outHandle.set(value);
        if(status != MS::kSuccess) return status;

        return MStatus::kSuccess;
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
    MStatus setPlugValue(MPlug& plug, Vec3_cu value);
    MStatus getPlugValue(const MPlug& plug, float& x, float& y, float& z);

    /*
     * Get a user interface from a node, with type checking.  On failure, return NULL.
     *
     * ImplicitSurface *surface = DagHelpers::GetInterfaceFromNode<ImplicitSurface>(node, &status);
     */
    template<typename T>
    T *GetInterfaceFromNode(MObject &node, MStatus *status)
    {
        MFnDependencyNode plugDep(node, status);
        if(*status != MS::kSuccess) return NULL;

        MTypeId expectedTypeId = T::id;
        MTypeId actualTypeId = plugDep.typeId(status);
        if(*status != MS::kSuccess) return NULL;
        if(expectedTypeId != actualTypeId)
        {
            *status = MStatus::kFailure;
            status->perror("Node has the wrong type");
            return NULL;
        }

        T *obj = (T *) plugDep.userNode(status);
        if(*status != MS::kSuccess) return NULL;

        return obj;
    }

    // This holds the dependencies of an object.  We can't use a simple std::map to do this, since
    // there's nothing on MObject for us to create an ordering from.
/*    class MObjectDependency
    {
        MObject from;
        std::vector<MObject> to;
    };
    */

    // MObject doesn't define operator<, and there's nothing public on MObject to allow ordering
    // them.  This makes it tricky to put them in std::map and std::set.  Implement a simple MFn
    // that does nothing but return the base pointer of an MObject, which is the implementation
    // pointer which is the same for all MObjects pointing at the same underlying object.  We can't
    // do much with this, since it's an opaque pointer, but we can use it to compare two MObjects.
    class FnGetBasePtr: public MFnBase
    {
    public:
            FnGetBasePtr(MObject obj) { setObject(obj); }
            const MPtrBase *ptr() const { return f_constptr; }
    };

    // Compare two MObjects, to allow them to be stored in standard containers.
    struct MObjectComparitor
    {
        bool operator()(const MObject &lhs, const MObject &rhs) const
        {
            FnGetBasePtr lhsPtr(lhs);
            FnGetBasePtr rhsPtr(rhs);
            return lhsPtr.ptr() < rhsPtr.ptr();
        }
    };

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
        void add(MObject from, MObject to);

        // Apply all dependencies.
        MStatus apply();

        // Return true if dest is affected by source.
        //
        // This is true if add(source, dest) was called, or if source affects any attribute
        // that recursively affects dest.
        bool isAffectedBy(MObject source, MObject dest) const;

    private:
        MStatus apply_one(MObject from, MObject to, set<MObject, MObjectComparitor> &stack);

        // MObject isn't ordered, so we can't use it in a map normally.  Since we're only using this with static attribute
        // objects, we can make sure that we only ever have a single instance of the MObject, as long as we're careful to
        // not make copies.
        map<MObject, set<MObject, MObjectComparitor>, MObjectComparitor> dependencies;

        // Get an array of attributes affected by obj.
        //
        // add() is always given pointers to the static attribute MObjects created in ::initialize.  However,
        // isAffectedBy() is typically given copies of the MObject, eg. from plug.attribute() within compute().
        // They won't have the same pointer, so we need to find them by equality.
        MObjectArray getObjectsAffectedBy(MObject obj) const;
    };
}

#endif
