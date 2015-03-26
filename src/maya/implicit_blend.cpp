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
#include "utils/misc_utils.hpp"

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
MObject ImplicitBlend::meshGeometryUpdateAttr;
MObject ImplicitBlend::worldImplicit;
MObject ImplicitBlend::previewIso;

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
    return handle_exceptions([&] {
        MStatus status = MStatus::kSuccess;

        MFnNumericAttribute numAttr;
        MFnCompoundAttribute cmpAttr;
        MFnTypedAttribute typedAttr;

        meshGeometryUpdateAttr = numAttr.create("meshGeometryUpdate", "meshGeometryUpdate", MFnNumericData::Type::kInt, 0, &status);
        numAttr.setStorable(false);
        numAttr.setHidden(true);
        addAttribute(meshGeometryUpdateAttr);

        previewIso = numAttr.create("previewIso", "previewIso", MFnNumericData::Type::kFloat, 0.5f, &status);
        addAttribute(previewIso);
        dependencies.add(previewIso, meshGeometryUpdateAttr);

        // Note that this attribute isn't set to worldSpace.  The input surfaces are world space, and the
        // output combined surfaces are world space, but we ignore the position of this actual node.
        worldImplicit = typedAttr.create("worldImplicit", "worldImplicit", ImplicitSurfaceData::id, MObject::kNullObj, &status);
        typedAttr.setUsesArrayDataBuilder(true);
        typedAttr.setWritable(false);
        addAttribute(worldImplicit);
        dependencies.add(ImplicitBlend::worldImplicit, ImplicitBlend::meshGeometryUpdateAttr);

        implicit = typedAttr.create("implicit", "implicit", ImplicitSurfaceData::id, MObject::kNullObj, &status);
        typedAttr.setReadable(false);
        dependencies.add(ImplicitBlend::implicit, ImplicitBlend::worldImplicit);
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
        dependencies.add(ImplicitBlend::surfaces, ImplicitBlend::worldImplicit);

        status = dependencies.apply(); merr("dependencies.apply");
    });
}

MStatus ImplicitBlend::setDependentsDirty(const MPlug &plug_, MPlugArray &plugArray)
{
    return handle_exceptions_ret([&] {
        MStatus status = MStatus::kSuccess;

        MPlug plug(plug_);

        // If the plug that was changed is a child, eg. point[0].x, move up to the parent
        // compound plug, eg. point[0].
        if(plug.isChild()) {
            plug = plug.parent(&status); merr("plug.parent");
        }

        // The rendered geometry is based on meshGeometryUpdateAttr.  If the node that was changed
        // affects that, then tell Maya that it needs to redraw the geometry.  This will
        // trigger ImplicitSurfaceGeometryOverride::updateDG, etc. if the shape is visible.
        // It looks like setAffectsAppearance() on meshGeometryUpdateAttr should do this for
        // us, but that doesn't seem to work.
        MObject node = plug.attribute();
        if(dependencies.isAffectedBy(node, ImplicitBlend::meshGeometryUpdateAttr)) {
            childChanged(kBoundingBoxChanged);
            MHWRender::MRenderer::setGeometryDrawDirty(thisMObject());
        }

        return MPxSurfaceShape::setDependentsDirty(plug, plugArray);
    });
}

// Remember whether implicit inputs are connected, since Maya doesn't clear them on
// disconnection.
MStatus ImplicitBlend::connectionMade(const MPlug &plug, const MPlug &otherPlug, bool asSrc)
{
    return handle_exceptions_ret([&] {
        MStatus status = MStatus::kSuccess;
        if(!asSrc && plug == ImplicitBlend::implicit) {
            MPlug arrayPlug = plug.parent(&status); merr("plug.parent");
            implicitConnectedIndexes.insert(arrayPlug.logicalIndex());
        }
        return MPxSurfaceShape::connectionMade(plug, otherPlug, asSrc);
    });
}

MStatus ImplicitBlend::connectionBroken(const MPlug &plug, const MPlug &otherPlug, bool asSrc)
{
    return handle_exceptions_ret([&] {
        MStatus status = MStatus::kSuccess;
        if(!asSrc && plug == ImplicitBlend::implicit) {
            MPlug arrayPlug = plug.parent(&status); merr("plug.parent");
            implicitConnectedIndexes.erase(arrayPlug.logicalIndex());
        }

        return MPxSurfaceShape::connectionBroken(plug, otherPlug, asSrc);
    });
}

MStatus ImplicitBlend::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    return handle_exceptions_ret([&] {
        if(plug == worldImplicit) load_world_implicit(plug, dataBlock);
        else if(plug == meshGeometryUpdateAttr) load_mesh_geometry(dataBlock);
        return MStatus::kUnknownParameter;
    });
}

const MeshGeom &ImplicitBlend::get_mesh_geometry()
{
    // Update and return meshGeometry for the preview renderer.
    MStatus status = MStatus::kSuccess;
    MDataBlock dataBlock = forceCache();
    dataBlock.inputValue(ImplicitBlend::meshGeometryUpdateAttr, &status);
    return meshGeometry;
}

// On meshGeometryUpdateAttr, update meshGeometry.
void ImplicitBlend::load_mesh_geometry(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    dataBlock.inputValue(ImplicitBlend::worldImplicit, &status); merr("inputValue(worldImplicit)");

    float iso = DagHelpers::readHandle<float>(dataBlock, ImplicitBlend::previewIso, &status); merr("readHandle(previewIso)")

    meshGeometry = MeshGeom();

    // If we have no skeleton, just clear the geometry.
    if(skeleton.get() == NULL)
        return;

    skeleton->update_bones_data();
    MarchingCubes::compute_surface(meshGeometry, skeleton.get(), iso);
}

// Retrieve the list of input bones and their parents from our attributes.
void ImplicitBlend::get_input_bones(MDataBlock &dataBlock,
    std::vector<shared_ptr<const Bone> > &bones, std::vector<Bone::Id> &parents) const
{
    MStatus status = MStatus::kSuccess;

    // Retrieve our input surfaces.  This will also update their transforms, etc. if needed.
    MArrayDataHandle surfacesHandle = dataBlock.inputArrayValue(ImplicitBlend::surfaces, &status); merr("inputArrayValue(surfaces)");

    // Create a list of our input surfaces and their relationships.
    map<int, shared_ptr<const Skeleton> > logicalIndexToImplicitBones;
    map<int,int> logicalIndexToParentIndex;
    for(int i = 0; i < (int) surfacesHandle.elementCount(); ++i)
    {
        status = surfacesHandle.jumpToElement(i); merr("surfacesHandle.jumpToElement");

        int logicalIndex = surfacesHandle.elementIndex(&status); merr("surfacesHandle.elementIndex");

        // If this node isn't actually connected, don't read it.  Maya doesn't reset MPxData when
        // connections are removed.
        if(implicitConnectedIndexes.find(logicalIndex) == implicitConnectedIndexes.end())
            continue;

        MDataHandle implicitHandle = surfacesHandle.inputValue(&status).child(ImplicitBlend::implicit); merr("surfacesHandle.inputValue");
        ImplicitSurfaceData *implicitSurfaceData = (ImplicitSurfaceData *) implicitHandle.asPluginData();
        logicalIndexToImplicitBones[logicalIndex] = implicitSurfaceData->getSkeleton();

        MDataHandle parentJointHandle = surfacesHandle.inputValue(&status).child(ImplicitBlend::parentJoint); merr("surfacesHandle.inputValue");
        int parentIdx = DagHelpers::readHandle<int>(parentJointHandle, &status); merr("readHandle(parentJointHandle)");

        logicalIndexToParentIndex[logicalIndex] = parentIdx;
    }

    // Check for out of bounds parent indexes.
    for(auto &it: logicalIndexToParentIndex)
    {
        if(logicalIndexToParentIndex.find(it.second) == logicalIndexToParentIndex.end())
            it.second = -1;
    }

    // Get the hierarchy order of the inputs, so we can create parents before children.
    vector<int> hierarchyOrder;
    if(!MiscUtils::getHierarchyOrder(logicalIndexToParentIndex, hierarchyOrder)) {
        // The input contains cycles.
        MDagPath dagPath = MDagPath::getAPathTo(thisMObject(), &status); merr("getAPathTo");
        MString path = dagPath.partialPathName(&status); merr("partialPathName");
        throw runtime_error(string("The ImplicitBlend node ") + path.asChar() + " contains cycles.");
    }

    // Each entry in logicalIndexToImplicitBones represents a Skeleton.  These will usually be skeletons with
    // just a single bone, representing an ImplicitSurface, but they can also have multiple bones,
    // if the input is another ImplicitBlend.  Add all bones in the input into our skeleton.
    // We can have the same bone more than once, if multiple skeletons give it to us, but a skeleton
    // can never have the same bone more than once.

    // firstBonePerSkeleton[n] is the index of the first bone (in bones) added for implicitBones[n].
    map<int,int> firstBonePerSkeleton;
    for(int idx: hierarchyOrder)
    {
        shared_ptr<const Skeleton> subSkeleton = logicalIndexToImplicitBones.at(idx);

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
        firstBonePerSkeleton[idx] = firstBoneIdx;

        int surfaceParentIdx = logicalIndexToParentIndex.at(idx);
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
                parentBoneIdx = firstBonePerSkeleton.at(surfaceParentIdx);
            }

            parents.push_back(parentBoneIdx);
        }
    }
}

void ImplicitBlend::update_skeleton(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    std::vector<shared_ptr<const Bone> > bones;
    std::vector<Bone::Id> parents;
    get_input_bones(dataBlock, bones, parents);

    // If the actual bones and their parenting hasn't changed, we're already up to date.
    if(bones == lastImplicitBones && parents == lastParents)
        return;

    lastImplicitBones = bones;
    lastParents = parents;

    // Skeletons can't have zero bones, so don't create one if we have no data.
    if(bones.size() == 0) {
        skeleton.reset();
        return;
    }

    // Create a skeleton containing the bones, replacing any previous skeleton.
    skeleton.reset(new Skeleton(bones, parents));
}

// Load the parameters associated with our input skeletons.  This is done on
// every update.
void ImplicitBlend::update_skeleton_params(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    if(skeleton.get() == NULL)
        return;

    MArrayDataHandle surfacesHandle = dataBlock.inputArrayValue(ImplicitBlend::surfaces, &status); merr("inputArrayValue(surfaces)");

    // Propagate each source bone's blending properties to our skeleton.  These are set
    // per-bone, but are stored on the skeleton.
    for(int i = 0; i < (int) surfacesHandle.elementCount(); ++i)
    {
        status = surfacesHandle.jumpToElement(i); merr("surfacesHandle.jumpToElement");
        int logicalIndex = surfacesHandle.elementIndex(&status); merr("surfacesHandle.elementIndex");

        if(implicitConnectedIndexes.find(logicalIndex) == implicitConnectedIndexes.end())
            continue;

        MDataHandle implicitHandle = surfacesHandle.inputValue(&status).child(ImplicitBlend::implicit); merr("surfacesHandle.inputValue");

        ImplicitSurfaceData *implicitSurfaceData = (ImplicitSurfaceData *) implicitHandle.asPluginData();
        const Skeleton *skel = implicitSurfaceData->getSkeleton().get();

        for(Bone::Id boneId: skel->get_bone_ids())
        {
            const Bone *srcBone = skel->get_bone(boneId).get();

            EJoint::Joint_t blending = skel->joint_blending(boneId);
            skeleton->set_joint_blending(boneId, blending);

            float bulgeMagnitude = skel->get_joints_bulge_magnitude(boneId);
            skeleton->set_joint_bulge_mag(boneId, bulgeMagnitude);

            IBL::Ctrl_setup controller = skel->get_joint_controller(boneId);
            skeleton->set_joint_controller(boneId, controller);
        }
    }
}

void ImplicitBlend::load_world_implicit(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    update_skeleton(dataBlock);
    update_skeleton_params(dataBlock);

    // Set ImplicitBlend::worldImplicit to our skeleton.  This may be NULL.
    status = setImplicitSurfaceData(dataBlock, ImplicitBlend::worldImplicit, skeleton); merr("setImplicitSurfaceData");
}


bool ImplicitBlend::isBounded() const { return true; }

MBoundingBox ImplicitBlend::boundingBox() const
{
    // Our bounding box is the union of our bones' bounding boxes.
    BBox_cu bbox;
    if(skeleton.get() != NULL) {
        for(Bone::Id boneId: skeleton->get_bone_ids())
        {
            const Bone *bone = skeleton->get_bone(boneId).get();

            // Get the surface bounding box in world space.
            BBox_cu boneBbox = bone->get_bbox(true, true);
            bbox = bbox.bbox_union(boneBbox);
        }
    }

    Point_cu top = bbox.get_corner(0);
    Point_cu bottom = bbox.get_corner(7);

    return MBoundingBox(MPoint(top.x, top.y, top.z), MPoint(bottom.x, bottom.y, bottom.z));
}


