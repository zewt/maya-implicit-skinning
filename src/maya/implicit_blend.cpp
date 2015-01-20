#define NO_CUDA

#include "implicit_blend.hpp"

#include <maya/MGlobal.h> 
#include <maya/MArrayDataBuilder.h>
#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixData.h>
#include <maya/MFnPluginData.h>
#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>

#include "maya/maya_helpers.hpp"
#include "maya/maya_data.hpp"

#include "skeleton.hpp"

#include <string.h>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include <map>
using namespace std;

MTypeId ImplicitBlend::id(0xEA119);
void *ImplicitBlend::creator() { return new ImplicitBlend(); }
DagHelpers::MayaDependencies ImplicitBlend::dependencies;

MObject ImplicitBlend::surfaces;
MObject ImplicitBlend::parentJoint;
MObject ImplicitBlend::implicit;
MObject ImplicitBlend::worldImplicit;

namespace {
    MStatus setImplicitSurfaceData(MDataBlock &dataBlock, MObject attr, shared_ptr<const Skeleton> skel)
    {
        MStatus status = MStatus::kSuccess;

        MFnPluginData dataCreator;
        dataCreator.create(ImplicitSurfaceData::id, &status);
        if(status != MS::kSuccess) return status;

        ImplicitSurfaceData *data = (ImplicitSurfaceData *) dataCreator.data(&status);
        if(status != MS::kSuccess) return status;

        data->setSkeleton(skel);

        MDataHandle worldImplicitHandle = dataBlock.outputValue(attr, &status);
        if(status != MS::kSuccess) return status;

        status = worldImplicitHandle.set(data);
        if(status != MS::kSuccess) return status;

        return MStatus::kSuccess;
    }
}

MStatus ImplicitBlend::initialize()
{
    MStatus status = MStatus::kSuccess;

    MFnNumericAttribute numAttr;
    MFnCompoundAttribute cmpAttr;
    MFnTypedAttribute typedAttr;

    // Note that this attribute isn't set to worldSpace.  The input surfaces are world space, and the
    // output combined surfaces are world space, but we ignore the position of this actual node.
    worldImplicit = typedAttr.create("worldImplicit", "worldImplicit", ImplicitSurfaceData::id, MObject::kNullObj, &status);
    if(status != MS::kSuccess) return status;
    typedAttr.setUsesArrayDataBuilder(true);
    typedAttr.setWritable(false);
    addAttribute(worldImplicit);

    implicit = typedAttr.create("implicit", "implicit", ImplicitSurfaceData::id, MObject::kNullObj, &status);
    if(status != MS::kSuccess) return status;
    typedAttr.setReadable(false);
    dependencies.add(implicit, worldImplicit);
    addAttribute(implicit);

    parentJoint = numAttr.create("parentIdx", "parentIdx", MFnNumericData::Type::kInt, -1, &status);
    addAttribute(parentJoint);
    dependencies.add(parentJoint, worldImplicit);

    surfaces = cmpAttr.create("surfaces", "surfaces", &status);
    cmpAttr.setReadable(false);
    cmpAttr.setArray(true);
    cmpAttr.addChild(implicit);
    cmpAttr.addChild(parentJoint);
    addAttribute(surfaces);

    status = dependencies.apply();
    if(status != MS::kSuccess) return status;

    return MStatus::kSuccess;
}

MStatus ImplicitBlend::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    if(plug == worldImplicit) return load_world_implicit(plug, dataBlock);
    return MStatus::kUnknownParameter;
}

// Given a list of parent indexes, return a list of indexes sorted from parent
// to child.
//
// For example, the input [3,-1,0,1] indicates that node 0's parent is node 3,
// node 1 is the root node, node 2's parent is node 0, and node 3's parent is
// node 0.
//
// The result of this hierarchy is [1,2,3,0].  Root nodes come first, followed
// by their children.
//
// The order of sibling nodes is unspecified.  Nodes with parent indexes that
// don't exist in the array are treated as root nodes.
//
// If the input contains cycles, the output will be empty and false will be returned.
// Nodes 
bool addHierarchyOrderRecursive(const map<int, vector<int> > children, vector<int> &out, int rootIdx, set<int> &indexesOnStack)
{
    // If this index is already on the stack, then the input has a cycle.
    if(indexesOnStack.find(rootIdx) != indexesOnStack.end())
        return false;

    indexesOnStack.insert(rootIdx);

    // Process this node before its children.
    out.push_back(rootIdx);

    // Recurse through all children of this node.
    bool result = true;
    if(children.find(rootIdx) != children.end()) {
        const vector<int> &childIndexes = children.at(rootIdx);
        for(int childIdx: childIndexes)
        {
            if(!addHierarchyOrderRecursive(children, out, childIdx, indexesOnStack))
                result = false;
        }
    }

    indexesOnStack.erase(rootIdx);
    return true;
}

bool getHierarchyOrder(const vector<int> &parent, vector<int> &out)
{
    // Make a list of each node's children.
    map<int, vector<int> > children;
    for(int idx = 0; idx < (int) parent.size(); ++idx) {
        int parent_idx = parent[idx];
        if(parent_idx >= 0 && parent_idx < parent.size())
            children[parent_idx].push_back(idx);
    }

    // Start processing root nodes.
    set<int> indexesOnStack;
    for(int idx = 0; idx < (int) parent.size(); ++idx)
    {
        int parent_idx = parent[idx];
        if(parent_idx >= 0 && parent_idx < parent.size())
            continue;

        if(!addHierarchyOrderRecursive(children, out, idx, indexesOnStack))
        {
            out.clear();
            return false;
        }
    }
    return true;
}

MStatus ImplicitBlend::update_skeleton(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // Retrieve our input surfaces.  This will also update their transforms, etc. if needed.
    MArrayDataHandle surfacesHandle = dataBlock.inputArrayValue(ImplicitBlend::surfaces, &status);
    if(status != MS::kSuccess) return status;

    // Create a list of our input surfaces and their relationships.
    vector<shared_ptr<const Skeleton> > implicitBones;
    vector<int> surfaceParents;
    for(int i = 0; i < (int) surfacesHandle.elementCount(); ++i)
    {
        status = surfacesHandle.jumpToElement(i);
        if(status != MS::kSuccess) return status;

        int logicalIndex = surfacesHandle.elementIndex(&status);
        if(status != MS::kSuccess) return status;

        implicitBones.resize(max(logicalIndex+1, (int) implicitBones.size()));
        surfaceParents.resize(max(logicalIndex+1, (int) surfaceParents.size()));

        MDataHandle implicitHandle = surfacesHandle.inputValue(&status).child(ImplicitBlend::implicit);
        if(status != MS::kSuccess) return status;

        ImplicitSurfaceData *implicitSurfaceData = (ImplicitSurfaceData *) implicitHandle.asPluginData();
        implicitBones[logicalIndex] = implicitSurfaceData->getSkeleton();

        MDataHandle parentJointHandle = surfacesHandle.inputValue(&status).child(ImplicitBlend::parentJoint);
        if(status != MS::kSuccess) return status;

        int parentIdx = DagHelpers::readHandle<int>(parentJointHandle, &status);
        if(status != MS::kSuccess) return status;

        surfaceParents[logicalIndex] = parentIdx;
    }

    // If the actual bones and their parenting hasn't changed, we're already up to date.
    if(implicitBones == lastImplicitBones && surfaceParents == lastParents)
        return MStatus::kSuccess;

    lastImplicitBones = implicitBones;
    lastParents = surfaceParents;

    // Get the hierarchy order of the inputs, so we can create parents before children.
    vector<int> hierarchyOrder;
    if(!getHierarchyOrder(surfaceParents, hierarchyOrder)) {
        // The input contains cycles.
        MDagPath dagPath = MDagPath::getAPathTo(thisMObject(), &status);
        if(status != MS::kSuccess) return status;

        MString path = dagPath.partialPathName(&status);
        if(status != MS::kSuccess) return status;
        MGlobal::displayError("The ImplicitBlend node " + path + " contains cycles.");
        return MStatus::kSuccess;
    }

    // Each entry in implicitBones represents a Skeleton.  These will usually be skeletons with
    // just a single bone, representing an ImplicitSurface, but they can also have multiple bones,
    // if the input is another ImplicitBlend.  Add all bones in the input into our skeleton.
    // We can have the same bone more than once, if multiple skeletons give it to us, but a skeleton
    // can never have the same bone more than once.
    std::vector<shared_ptr<const Bone> > bones;
    std::vector<Bone::Id> parents;

    // firstBonePerSkeleton[n] is the index of the first bone (in bones) added for implicitBones[n].
    std::vector<int> firstBonePerSkeleton(surfaceParents.size(), -1);
    for(int i = 0; i < (int) hierarchyOrder.size(); ++i)
    {
        int idx = hierarchyOrder[i];
        shared_ptr<const Skeleton> subSkeleton = implicitBones[idx];
        int surfaceParentIdx = surfaceParents[idx];

        int firstBoneIdx = -1;

        // Add all of the bones.
        map<Bone::Id,int> boneIdToIdx;
        for(Bone::Id boneId: subSkeleton->get_bone_ids())
        {
            shared_ptr<const Bone> bone = subSkeleton->get_bone(boneId);
            firstBoneIdx = (int) bones.size();
            boneIdToIdx[bone->get_bone_id()] = (int) bones.size();
            bones.push_back(bone);
        }

        for(Bone::Id boneId: subSkeleton->get_bone_ids())
        {
            int parentBoneIdx = -1;

            Bone::Id parentBoneId = subSkeleton->parent(boneId);
            if(parentBoneId != -1)
            {
                // If the bone within the subskeleton has a parent, it's another bone in the same
                // skeleton.  Add the parent bone's index within bones as the parent.
                parentBoneIdx = boneIdToIdx.at(parentBoneId);
            }
            else if(surfaceParentIdx != -1)
            {
                // This bone is at the root of its skeleton.  Use the first bone of the parent
                // surface.  If the parent surface doesn't actually have any bones, leave this
                // as a root joint.  It's guaranteed that we've already created the parent,
                // since we're traversing in hierarchy order.
                parentBoneIdx = firstBonePerSkeleton[surfaceParentIdx];
            }

            parents.push_back(parentBoneIdx);
        }

        firstBonePerSkeleton[idx] = firstBoneIdx;
    }

    // Skeletons can't have zero bones, so don't create one if we have no data.
    if(bones.size() == 0) {
        skeleton.skel.reset();
        return MStatus::kSuccess;
    }

    // Create a skeleton containing the bones, replacing any previous skeleton.
    skeleton.skel.reset(new Skeleton(bones, parents));

    return MStatus::kSuccess;
}

MStatus ImplicitBlend::load_world_implicit(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    status = update_skeleton(dataBlock);
    if(status != MS::kSuccess) return status;

    if(skeleton.skel.get() != NULL) {
        // Update our skeleton based on the bone data.  This lets the skeleton know that the bones
        // may have changed orientation.
        skeleton.skel->update_bones_data();
    }

    // Set ImplicitBlend::worldImplicit to our skeleton.  This may be NULL.
    status = setImplicitSurfaceData(dataBlock, ImplicitBlend::worldImplicit, skeleton.skel);
    if(status != MS::kSuccess) return status;

    return MStatus::kSuccess;
}



