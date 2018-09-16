#define NO_CUDA

#include "implicit_surface.hpp"

#include <string.h>
#include <math.h>
#include <assert.h>

#include <maya/MGlobal.h> 
#include <maya/MPxData.h>
#include <maya/MPxTransform.h>

#include <maya/MPxSurfaceShape.h>
#include <maya/MArrayDataBuilder.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnEnumAttribute.h>

#include <maya/MFnPluginData.h>
#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MFnMesh.h>

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>

#include "maya/maya_helpers.hpp"
#include "maya/maya_data.hpp"

#include "skeleton.hpp"

#include "implicit_surface_data.hpp"

#include <algorithm>
#include <map>
using namespace std;


namespace {
    void setImplicitSurfaceData(const MPlug &plug, MDataBlock &dataBlock, shared_ptr<const Skeleton> skel)
    {
        MStatus status = MStatus::kSuccess;

        MFnPluginData dataCreator;
        dataCreator.create(ImplicitSurfaceData::id, &status); merr("dataCreator(ImplicitSurfaceData)");
        ImplicitSurfaceData *data = (ImplicitSurfaceData *) dataCreator.data(&status); merr("dataCreator.data");

        data->setSkeleton(skel);

        int logicalIndex = plug.logicalIndex(&status); merr("logicalIndex()");
        status = DagHelpers::addObjectToArray(dataBlock, plug.attribute(), logicalIndex, data); merr("addObjectToArray");
    }
}















MTypeId ImplicitSurface::id(0xEA117);

MObject ImplicitSurface::hrbfRadiusAttr;
MObject ImplicitSurface::samplePointAttr;
MObject ImplicitSurface::sampleNormalAttr;
MObject ImplicitSurface::initialDir;
MObject ImplicitSurface::sampleSetUpdateAttr;
MObject ImplicitSurface::meshGeometryUpdateAttr;
MObject ImplicitSurface::worldImplicit;
MObject ImplicitSurface::blendMode;
MObject ImplicitSurface::bulgeStrength;

DagHelpers::MayaDependencies ImplicitSurface::dependencies;

MStatus ImplicitSurface::initialize()
{
    return handle_exceptions([&] {
        MStatus status = MStatus::kSuccess;

        MFnMatrixAttribute mAttr;
        MFnNumericAttribute numAttr;
        MFnCompoundAttribute cmpAttr;
        MFnTypedAttribute typedAttr;
        MFnEnumAttribute enumAttr;

        samplePointAttr = numAttr.create("point", "p", MFnNumericData::Type::k3Float, 0, &status);
        numAttr.setArray(true);
        addAttribute(samplePointAttr);

        sampleNormalAttr = numAttr.create("normal", "n", MFnNumericData::Type::k3Float, 0, &status);
        numAttr.setArray(true);
        addAttribute(sampleNormalAttr);

        hrbfRadiusAttr = numAttr.create("hrbfRadius", "hrbfRadius", MFnNumericData::Type::kFloat, 0, &status);
        addAttribute(hrbfRadiusAttr);

        sampleSetUpdateAttr = numAttr.create("sampleSetUpdate", "sampleSetUpdate", MFnNumericData::Type::kInt, 0, &status);
        numAttr.setStorable(false);
        numAttr.setHidden(true);
        addAttribute(sampleSetUpdateAttr);

        dependencies.add(ImplicitSurface::samplePointAttr, ImplicitSurface::sampleSetUpdateAttr);
        dependencies.add(ImplicitSurface::sampleNormalAttr, ImplicitSurface::sampleSetUpdateAttr);
        dependencies.add(ImplicitSurface::hrbfRadiusAttr, ImplicitSurface::sampleSetUpdateAttr);

        meshGeometryUpdateAttr = numAttr.create("meshGeometryUpdate", "meshGeometryUpdate", MFnNumericData::Type::kInt, 0, &status);
        numAttr.setStorable(false);
        numAttr.setHidden(true);
        addAttribute(meshGeometryUpdateAttr);
        dependencies.add(ImplicitSurface::sampleSetUpdateAttr, ImplicitSurface::meshGeometryUpdateAttr);

        blendMode = enumAttr.create("blendMode", "blendMode", 0, &status);
        enumAttr.addField("Max", 0);
        enumAttr.addField("Bulge", 1);
        enumAttr.addField("Circle", 2);
        addAttribute(blendMode);

        bulgeStrength = numAttr.create("bulgeStrength", "bulgeStrength", MFnNumericData::Type::kFloat, 0.7, &status);
        addAttribute(bulgeStrength);

        worldImplicit = typedAttr.create("worldImplicit", "worldImplicit", ImplicitSurfaceData::id, MObject::kNullObj, &status);
        typedAttr.setWritable(false);
        typedAttr.setStorable(false);
        typedAttr.setUsesArrayDataBuilder(true);
        typedAttr.setArray(true); // We don't actually want an array, but setWorldSpace only works with arrays
        status = typedAttr.setWorldSpace(true);
        addAttribute(worldImplicit);

        dependencies.add(ImplicitSurface::blendMode, ImplicitSurface::worldImplicit);
        dependencies.add(ImplicitSurface::bulgeStrength, ImplicitSurface::worldImplicit);
        dependencies.add(ImplicitSurface::sampleSetUpdateAttr, ImplicitSurface::worldImplicit);

        initialDir = numAttr.create("initialDir", "initialDir", MFnNumericData::Type::k3Float, 0, &status);
        numAttr.setInternal(true);
        addAttribute(initialDir);

        status = dependencies.apply(); merr("dependencies.apply");
    });
}

MStatus ImplicitSurface::setDependentsDirty(const MPlug &plug_, MPlugArray &plugArray)
{
    return handle_exceptions_ret([&] {
        MStatus status = MStatus::kSuccess;

        MPlug plug(plug_);

        // If the plug that was changed is a child, eg. point[0].x, move up to the parent
        // compound plug, eg. point[0].
        if(plug.isChild()) {
            plug = plug.parent(&status); merr("parent");
        }

        // The rendered geometry is based on meshGeometryUpdateAttr.  If the node that was changed
        // affects that, then tell Maya that it needs to redraw the geometry.  This will
        // trigger ImplicitSurfaceGeometryOverride::updateDG, etc. if the shape is visible.
        // It looks like setAffectsAppearance() on meshGeometryUpdateAttr should do this for
        // us, but that doesn't seem to work.
        MObject node = plug.attribute();
        if(dependencies.isAffectedBy(node, ImplicitSurface::meshGeometryUpdateAttr)) {
            childChanged(kBoundingBoxChanged);
            MHWRender::MRenderer::setGeometryDrawDirty(thisMObject());
        }

        return MPxSurfaceShape::setDependentsDirty(plug, plugArray);
    });
}

void *ImplicitSurface::creator() { return new ImplicitSurface(); }
void ImplicitSurface::postConstructor()
{
    handle_exceptions([&] {
        bone.reset(new Bone());

        // Create a small, dummy Skeleton that contains just our bone.
        std::vector<shared_ptr<const Bone> > bone_list(1, bone);
        std::vector<int> parents(1, -1);
        boneSkeleton.reset(new Skeleton(bone_list, parents, true));

        setRenderable(true);

        MStatus status = MStatus::kSuccess;
        MPlug meshUpdatePlug(thisMObject(), worldImplicit);

        meshUpdatePlug.elementByLogicalIndex(0, &status);
    //    childPlug.setInt(0);
    });
}

MStatus ImplicitSurface::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    return handle_exceptions_ret([&] {
        MStatus status = MStatus::kSuccess;

        // If we're calculating the output geometry, use the default implementation, which will
        // call deform().
        printf("Compute: %s\n", plug.name().asChar());
        if(plug == sampleSetUpdateAttr) load_sampleset(dataBlock);
        else if(plug == meshGeometryUpdateAttr) load_mesh_geometry(dataBlock);
        else if(plug == worldImplicit) load_world_implicit(plug, dataBlock);
        else return MStatus::kUnknownParameter;
        return MStatus::kSuccess;
    });
}

void ImplicitSurface::save_sampleset(const SampleSet::InputSample &inputSample)
{
    MStatus status = MStatus::kSuccess;

    // Save the samples.
    MPlug samplePointPlug(thisMObject(), ImplicitSurface::samplePointAttr);
    MPlug sampleNormalPlug(thisMObject(), ImplicitSurface::sampleNormalAttr);
    for(int sampleIdx = 0; sampleIdx < inputSample.nodes.size(); ++sampleIdx)
    {
        MPlug samplePlug = samplePointPlug.elementByLogicalIndex(sampleIdx, &status); merr("samplePointPlug.elementByLogicalIndex");

        status = DagHelpers::setPlugValue(samplePlug,
            inputSample.nodes[sampleIdx].x,
            inputSample.nodes[sampleIdx].y,
            inputSample.nodes[sampleIdx].z);
        merr("setPlugValue(samplePlug)");

        MPlug normalPlug = sampleNormalPlug.elementByLogicalIndex(sampleIdx, &status); merr("sampleNormalPlug.elementByLogicalIndex");

        status = DagHelpers::setPlugValue(normalPlug,
            inputSample.n_nodes[sampleIdx].x,
            inputSample.n_nodes[sampleIdx].y,
            inputSample.n_nodes[sampleIdx].z);
        merr("setPlugValue(normalPlug)");
    }
}

void ImplicitSurface::load_sampleset(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    SampleSet::InputSample inputSample;

    // Load the samples.
    MArrayDataHandle samplePointHandle = dataBlock.inputArrayValue(ImplicitSurface::samplePointAttr, &status); merr("inputArrayValue(samplePointAttr)");
    MArrayDataHandle sampleNormalHandle = dataBlock.inputArrayValue(ImplicitSurface::sampleNormalAttr, &status); merr("inputArrayValue(samplePointAttr)");

    // Load the HRBF radius.  This isn't really part of the sample set.
    {
        MDataHandle hrbfRadiusHandle = dataBlock.inputValue(ImplicitSurface::hrbfRadiusAttr, &status); merr("inputValue(hrbfRadiusAttr)");
        float hrbfRadius = hrbfRadiusHandle.asFloat();
        bone->set_hrbf_radius(hrbfRadius, boneSkeleton.get());
    }

    if(samplePointHandle.elementCount() != sampleNormalHandle.elementCount())
        throw std::runtime_error("Element count mismatch");

    for(int sampleIdx = 0; sampleIdx < (int) samplePointHandle.elementCount(); ++sampleIdx)
    {
        status = samplePointHandle.jumpToElement(sampleIdx); merr("samplePointHandle.jumpToElement");
        status = sampleNormalHandle.jumpToElement(sampleIdx); merr("sampleNormalHandle.jumpToElement");

        DagHelpers::simpleFloat3 samplePoint = DagHelpers::readArrayHandle<DagHelpers::simpleFloat3>(samplePointHandle, &status); merr("readArrayHandle(samplePointHandle)");
        DagHelpers::simpleFloat3 sampleNormal = DagHelpers::readArrayHandle<DagHelpers::simpleFloat3>(sampleNormalHandle, &status); merr("readArrayHandle(sampleNormalHandle)");

        inputSample.nodes.push_back(Point_cu(samplePoint.x, samplePoint.y, samplePoint.z));
        inputSample.n_nodes.push_back(Vec3_cu(sampleNormal.x, sampleNormal.y, sampleNormal.z));
    }

    // Load the InputSample into the bone.
    if(inputSample.nodes.empty())
    {
        bone->set_enabled(false);
    }
    else
    {
        // Solve/compute HRBF weights
        bone->set_enabled(true);
        bone->discard_precompute();
        bone->get_hrbf().init_coeffs(inputSample.nodes, inputSample.n_nodes);
        printf("update_bone_samples: Solved %i nodes\n", (int) inputSample.nodes.size());

        // Make sure the current transforms are applied now that we've changed the bone.
        // XXX: If this is needed, Bone should probably do this internally.
        // HRBF_env::apply_hrbf_transfos();
        Precomputed_prim::update_device_transformations();

        if(bone->get_type() == EBone::HRBF)
            bone->precompute(boneSkeleton.get());
    }
}

// On meshGeometryUpdateAttr, update meshGeometry.
void ImplicitSurface::load_mesh_geometry(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    dataBlock.inputValue(ImplicitSurface::sampleSetUpdateAttr, &status); merr("inputValue(sampleSetUpdate)");

    // Temporarily set the world space transform of the bone to identity, so we can calculate
    // the mesh in object space.  Our transform node will apply the world space transform.
    Transfo worldSpace = bone->get_world_space_matrix();
    set_world_space(Transfo::identity());

    meshGeometry = MeshGeom();
    boneSkeleton->update_bones_data();
    MarchingCubes::compute_surface(meshGeometry, boneSkeleton.get());

    // Set the transform of the bone back.
    set_world_space(worldSpace);
}

void ImplicitSurface::set_world_space(Transfo tr)
{
    // If the transform hasn't changed, don't update.
    if(bone->get_world_space_matrix().equal(tr))
        return;

    bone->set_world_space_matrix(tr);
}

bool ImplicitSurface::setInternalValueInContext(const MPlug &plug, const MDataHandle &dataHandle, MDGContext &ctx)
{
    MStatus status = MStatus::kSuccess;
    if(plug == initialDir)
    {
        // When initialDir changes, store the new value in the bone.  Don't return false; we still
        // want Maya to store the value in the datablock.
        DagHelpers::simpleFloat3 dir = DagHelpers::readHandle<DagHelpers::simpleFloat3>(dataHandle, &status);
        if(status != MS::kSuccess) return status;
    
        bone->set_object_space_dir(Vec3_cu(dir.x, dir.y, dir.z));
    }

    return MPxSurfaceShape::setInternalValueInContext(plug, dataHandle, ctx);
}

void ImplicitSurface::load_world_implicit(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // Update dependencies.
    dataBlock.inputValue(ImplicitSurface::sampleSetUpdateAttr, &status); merr("inputValue(sampleSetUpdate)");

    // Load the initialDir.  We don't actually need this to have access to it (it's already been
    // stored in the Bone by setInternalValueInContext), but we need to do this to let Maya know
    // that we're actually using the ImplicitSurface::initialDir dependency.
    dataBlock.inputValue(ImplicitSurface::initialDir, &status); merr("inputValue(initialDir)");

    int smoothMode = DagHelpers::readHandle<short>(dataBlock, ImplicitSurface::blendMode, &status); merr("readHandle(blendMode)");

    EJoint::Joint_t jointType = EJoint::Joint_t::GC_ARC_CIRCLE_TWEAK;

    // These values must match the values in initialize().
    switch(smoothMode)
    {
    case 0: jointType = EJoint::Joint_t::MAX; break;
    case 1: jointType = EJoint::Joint_t::BULGE; break;
    case 2: jointType = EJoint::Joint_t::GC_ARC_CIRCLE_TWEAK; break;
    }
    boneSkeleton->set_joint_blending(bone->get_bone_id(), jointType);

    float bulgeStrength = DagHelpers::readHandle<float>(dataBlock, ImplicitSurface::bulgeStrength, &status); merr("readHandle(bulgeStrength)");
    boneSkeleton->set_joint_bulge_mag(bone->get_bone_id(), bulgeStrength);

    MMatrix worldMatrix = getWorldMatrix(dataBlock, 0);
    Transfo worldMatrixTransfo = DagHelpers::MMatrixToTransfo(worldMatrix);
    set_world_space(worldMatrixTransfo);

    // Set the attribute to a ImplicitSurfaceData data node pointing at the skeleton.
    // The caller can use this to retrieve the skeleton.
    setImplicitSurfaceData(plug, dataBlock, boneSkeleton);
}

const MeshGeom &ImplicitSurface::get_mesh_geometry()
{
    // Update and return meshGeometry for the preview renderer.
    MStatus status = MStatus::kSuccess;
    MDataBlock dataBlock = forceCache();
    dataBlock.inputValue(ImplicitSurface::meshGeometryUpdateAttr, &status);
    return meshGeometry;
}

MStatus ImplicitSurface::set_bone_direction(Vec3_cu dir)
{
    bone->_dir = dir;
    bone->_length = bone->_dir.norm();

    // Save the direction to initialDir.
    MStatus status = MStatus::kSuccess;
    MPlug initialDirPlug(thisMObject(), ImplicitSurface::initialDir);
    status = DagHelpers::setPlugValue(initialDirPlug, dir.x, dir.y, dir.z);
    if(status != MStatus::kSuccess) return status;

    return MStatus::kSuccess;

}

bool ImplicitSurface::isBounded() const { return true; }

MBoundingBox ImplicitSurface::boundingBox() const
{
    // Get the surface bounding box in object space.
    BBox_cu bbox = bone->get_bbox(true, false);
    Point_cu top = bbox.get_corner(0);
    Point_cu bottom = bbox.get_corner(7);

    return MBoundingBox(MPoint(top.x, top.y, top.z), MPoint(bottom.x, bottom.y, bottom.z));
}

// XXX: It would be better to cast a ray down the selection to see if it intersects with the
// implicit surface.  This simple implementation just selects for any click in the bounding
// box.  Note that we use this for both ImplicitSurface and ImplicitBlend.
bool ImplicitSurfaceUI::select(MSelectInfo &selectInfo, MSelectionList &selectionList, MPointArray &worldSpaceSelectPts) const
{
    MSelectionList item;
    item.add(selectInfo.selectPath());

    MPoint point;
    selectInfo.addSelection(item, point, selectionList, worldSpaceSelectPts, MSelectionMask(MSelectionMask::kSelectObjectsMask), false);
    return true;
}
