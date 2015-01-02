#include <string.h>
#include <maya/MIOStream.h>
#include <math.h>
#include <assert.h>

// Don't include CUDA headers in this file.  CUDA and Maya headers are incompatible due to
// namespace conflicts.
#define NO_CUDA

#include <maya/MGlobal.h> 
#include <maya/MPxDeformerNode.h> 
#include <maya/MPxCommand.h>
#include <maya/MPxData.h>
#include <maya/MItGeometry.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixData.h>

#include <maya/MFnDagNode.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnTransform.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnGeometryFilter.h>
#include <maya/MFnPluginData.h>

#include <maya/MArgList.h>
#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MPlugArray.h>
#include <maya/MFnMesh.h>

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>

#include <maya/MPoint.h>
#include <maya/MPointArray.h>
#include <maya/MVector.h>
#include <maya/MVectorArray.h>
#include <maya/MMatrix.h>

#include <maya/MDagModifier.h>

#include "maya/maya_helpers.hpp"
#include "maya/maya_data.hpp"

#include "loader_mesh.hpp"
#include "loader_skel.hpp"
#include "skeleton.hpp"

#include "loader_mesh.hpp"
#include "sample_set.hpp"
#include "animated_mesh_ctrl.hpp"
#include "cuda_ctrl.hpp"

// #include "animesh.hpp"

#include <algorithm>
#include <map>
using namespace std;

// Note that functions which don't take a dataBlock but read attributes aren't intended to be called from
// compute().  Maya has a different API for reading attributes inside compute().
class ImplicitSkinDeformer: public MPxDeformerNode
{
public:
    static const MTypeId id;

    virtual ~ImplicitSkinDeformer() { }

    static void *creator() { return new ImplicitSkinDeformer(); }
    static MStatus initialize();
    
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock);
    MStatus deform(MDataBlock &block, MItGeometry &iter, const MMatrix &mat, unsigned int multiIndex);

    MStatus save_sampleset(const SampleSet::SampleSet &samples);
    MStatus set_bind_pose();

    MStatus load_skeleton(MDataBlock &dataBlock);
    MStatus load_sampleset(MDataBlock &dataBlock);
    MStatus load_mesh(MDataBlock &dataBlock);
    MStatus load_base_potential(MDataBlock &dataBlock);

    MStatus sample_all_joints();

    static MObject unskinnedGeomAttr;

    // The geomMatrix of the skinCluster at setup time.
    static MObject geomMatrixAttr;

    // Per-joint attributes:
    static MObject influenceJointsAttr;

    // The parent index of each joint.  This is relative to other entries in influenceJointsAttr.  Root
    // joints are set to -1.
    static MObject parentJointAttr;

    // Each joint's influenceBindMatrix at setup time.
    static MObject influenceBindMatrixAttr;

    // The world matrix of each influence.  This is usually linked to the worldMatrix attribute of the joint.
    static MObject influenceMatrixAttr;
    static MObject junctionRadiusAttr;
    static MObject samplePointAttr;
    static MObject sampleNormalAttr;

private:
    Cuda_ctrl::CudaCtrl cudaCtrl;

    // The *UpdateAttr attributes are for tracking when we need to update an internal data structure to reflect
    // a change made to the Maya attributes.  These aren't saved to disk or shown to the user.  For example, when
    // we set new data samplePointAttr, it will trigger sampleSetUpdateAttr's dependency on it, causing
    // us to update the SampleSet.  Triggering the sampleSetUpdateAttr dependency will then trigger
    // basePotentialUpdateAttr, causing us to update the base potential.  This allows data to be manipulated
    // normally within Maya, and we just update our data based on it as needed.

    // Internal dependency attributes:
    // Evaluated when we need to update a SampleSet and load it:
    static MObject sampleSetUpdateAttr;

    // Represents cudaCtrl._skeleton being up to date:
    static MObject skeletonUpdateAttr;

    // Represents cudaCtrl._anim_mesh being up to date with the skinned geometry.
    static MObject meshUpdateAttr;
    
    // Represents the base potential being up to date for the current SampleSet and unskinned mesh.
    static MObject basePotentialUpdateAttr;

    MStatus createSkeleton(MDataBlock &dataBlock, Loader::Abs_skeleton &skeleton);
    MStatus setGeometry(MDataHandle &inputGeomDataHandle);

};

// XXX: http://help.autodesk.com/view/MAYAUL/2015/ENU/?guid=__cpp_ref_class_m_type_id_html says that
// ADN assigns public blocks of IDs, but nothing says how to request a block without paying for
// a commercial ADN account.  Let's use a value in the devkit sample range, so it's unlikely to conflict,
// and it does, it won't conflict with somebody's internal-use IDs (0-0x7ffff).  At worst, we'll collide
// with a sample or somebody else doing the same thing.
const MTypeId ImplicitSkinDeformer::id(0xEA115);
MObject ImplicitSkinDeformer::unskinnedGeomAttr;
MObject ImplicitSkinDeformer::geomMatrixAttr;
MObject ImplicitSkinDeformer::influenceJointsAttr;
MObject ImplicitSkinDeformer::parentJointAttr;
MObject ImplicitSkinDeformer::influenceBindMatrixAttr;
MObject ImplicitSkinDeformer::influenceMatrixAttr;
MObject ImplicitSkinDeformer::junctionRadiusAttr;
MObject ImplicitSkinDeformer::sampleSetUpdateAttr;
MObject ImplicitSkinDeformer::skeletonUpdateAttr;
MObject ImplicitSkinDeformer::meshUpdateAttr;
MObject ImplicitSkinDeformer::basePotentialUpdateAttr;
MObject ImplicitSkinDeformer::samplePointAttr;
MObject ImplicitSkinDeformer::sampleNormalAttr;

static void loadDependency(MObject obj, MObject attr, MStatus *status)
{
    if(*status != MStatus::kSuccess)
        return;

    MPlug updatePlug(obj, attr);

    bool unused;
    *status = updatePlug.getValue(unused);
}

MStatus ImplicitSkinDeformer::initialize()
{
    MStatus status = MStatus::kSuccess;

    DagHelpers::MayaDependencies dep;

    // XXX
    // MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer blendNode weights;");

    MFnMatrixAttribute mAttr;
    MFnNumericAttribute numAttr;
    MFnCompoundAttribute cmpAttr;
    MFnTypedAttribute typeAttr;

    unskinnedGeomAttr = typeAttr.create("unskinnedGeom", "unskinnedGeom", MFnData::Type::kMesh, MObject::kNullObj, &status);
    addAttribute(unskinnedGeomAttr);

    geomMatrixAttr = mAttr.create("geomMatrix", "gm");
    addAttribute(geomMatrixAttr);

    influenceBindMatrixAttr = mAttr.create("influenceBindMatrix", "ibm");
    addAttribute(influenceBindMatrixAttr);

    parentJointAttr = numAttr.create("parentIdx", "parentIdx", MFnNumericData::Type::kInt, -1, &status);
    addAttribute(parentJointAttr);

    // The joint's output matrix.
    influenceMatrixAttr = mAttr.create("matrix", "ma");
    addAttribute(influenceMatrixAttr);

    // SampleSet:
    junctionRadiusAttr = numAttr.create("junctionRadius", "jr", MFnNumericData::Type::kFloat, 0, &status);
    addAttribute(junctionRadiusAttr);

    samplePointAttr = numAttr.create("point", "p", MFnNumericData::Type::k3Float, 0, &status);
    numAttr.setArray(true);
    addAttribute(samplePointAttr);

    sampleNormalAttr = numAttr.create("normal", "n", MFnNumericData::Type::k3Float, 0, &status);
    numAttr.setArray(true);
    addAttribute(sampleNormalAttr);

    // The main joint array:
    influenceJointsAttr = cmpAttr.create("joints", "jt", &status);
    cmpAttr.setArray(true);
    cmpAttr.addChild(influenceBindMatrixAttr);
    cmpAttr.addChild(parentJointAttr);
    cmpAttr.addChild(influenceMatrixAttr);
    cmpAttr.addChild(junctionRadiusAttr);
    cmpAttr.addChild(samplePointAttr);
    cmpAttr.addChild(sampleNormalAttr);
    addAttribute(influenceJointsAttr);

    sampleSetUpdateAttr = numAttr.create("sampleSetUpdate", "sampleSetUpdate", MFnNumericData::Type::kInt, 0, &status);
    numAttr.setStorable(false);
    numAttr.setHidden(true);
    addAttribute(sampleSetUpdateAttr);

    dep.add(ImplicitSkinDeformer::junctionRadiusAttr, ImplicitSkinDeformer::sampleSetUpdateAttr);
    dep.add(ImplicitSkinDeformer::influenceBindMatrixAttr, ImplicitSkinDeformer::sampleSetUpdateAttr);
    dep.add(ImplicitSkinDeformer::samplePointAttr, ImplicitSkinDeformer::sampleSetUpdateAttr);
    dep.add(ImplicitSkinDeformer::sampleNormalAttr, ImplicitSkinDeformer::sampleSetUpdateAttr);

    // Reloading the mesh recreates animesh, which resets bones, so if we reload the whole mesh we need to
    // reload the SampleSet as well.
    dep.add(ImplicitSkinDeformer::meshUpdateAttr, ImplicitSkinDeformer::sampleSetUpdateAttr);

    meshUpdateAttr = numAttr.create("meshUpdate", "meshUpdate", MFnNumericData::Type::kInt, 0, &status);
    numAttr.setStorable(false);
    numAttr.setHidden(true);
    addAttribute(meshUpdateAttr);
    dep.add(ImplicitSkinDeformer::unskinnedGeomAttr, ImplicitSkinDeformer::meshUpdateAttr);

    skeletonUpdateAttr = numAttr.create("skeletonUpdate", "skeletonUpdate", MFnNumericData::Type::kInt, 0, &status);
    numAttr.setStorable(false);
    numAttr.setHidden(true);
    addAttribute(skeletonUpdateAttr);
    if(status != MS::kSuccess) return status;

    dep.add(ImplicitSkinDeformer::parentJointAttr, ImplicitSkinDeformer::skeletonUpdateAttr);
    dep.add(ImplicitSkinDeformer::influenceBindMatrixAttr, ImplicitSkinDeformer::skeletonUpdateAttr);

    basePotentialUpdateAttr = numAttr.create("basePotentialUpdate", "basePotentialUpdate", MFnNumericData::Type::kInt, 0, &status);
    numAttr.setStorable(false);
    numAttr.setHidden(true);
    addAttribute(basePotentialUpdateAttr);

    dep.add(ImplicitSkinDeformer::meshUpdateAttr, ImplicitSkinDeformer::basePotentialUpdateAttr);
    dep.add(ImplicitSkinDeformer::sampleSetUpdateAttr, ImplicitSkinDeformer::basePotentialUpdateAttr);
    dep.add(ImplicitSkinDeformer::skeletonUpdateAttr, ImplicitSkinDeformer::basePotentialUpdateAttr);

    // All of the dependency nodes are required by the output geometry.
    dep.add(ImplicitSkinDeformer::geomMatrixAttr, ImplicitSkinDeformer::outputGeom);
    dep.add(ImplicitSkinDeformer::influenceMatrixAttr, ImplicitSkinDeformer::outputGeom);
    dep.add(ImplicitSkinDeformer::skeletonUpdateAttr, ImplicitSkinDeformer::outputGeom);
    dep.add(ImplicitSkinDeformer::meshUpdateAttr, ImplicitSkinDeformer::outputGeom);
    dep.add(ImplicitSkinDeformer::basePotentialUpdateAttr, ImplicitSkinDeformer::outputGeom);

    status = dep.apply();
    if(status != MS::kSuccess) return status;

    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // If we're calculating the output geometry, use the default implementation, which will
    // call deform().
    printf("Compute: %s\n", plug.name().asChar());
    if(plug.attribute() == outputGeom) return MPxDeformerNode::compute(plug, dataBlock);
    else if(plug.attribute() == sampleSetUpdateAttr) return load_sampleset(dataBlock);
    else if(plug.attribute() == skeletonUpdateAttr) return load_skeleton(dataBlock);
    else if(plug.attribute() == meshUpdateAttr) return load_mesh(dataBlock);
    else if(plug.attribute() == basePotentialUpdateAttr) return load_base_potential(dataBlock);
    else return MStatus::kUnknownParameter;
}

MStatus ImplicitSkinDeformer::deform(MDataBlock &dataBlock, MItGeometry &geomIter, const MMatrix &mat, unsigned int multiIndex)
{
    // We only support a single input, like skinCluster.
    if(multiIndex > 0)
        return MStatus::kSuccess;

    MStatus status = MStatus::kSuccess;

    // Read the dependency attributes that represent data we need.  We don't actually use the
    // results of inputvalue(); this is triggering updates for cudaCtrl data.
    dataBlock.inputValue(ImplicitSkinDeformer::basePotentialUpdateAttr, &status);
    if(status != MS::kSuccess) return status;

    dataBlock.inputValue(ImplicitSkinDeformer::skeletonUpdateAttr, &status);
    if(status != MS::kSuccess) return status;

    dataBlock.inputValue(ImplicitSkinDeformer::meshUpdateAttr, &status);
    if(status != MS::kSuccess) return status;

    // If we don't have a mesh to work with yet, stop.
    if(cudaCtrl._mesh == NULL)
        return MStatus::kSuccess;

    // Get the geomMatrixAttr attribute.
    MMatrix objectToWorldSpaceMat = DagHelpers::readHandle<MMatrix>(dataBlock, geomMatrixAttr, &status);
    if(status != MS::kSuccess) return status;

    MMatrix worldToObjectSpaceMat = objectToWorldSpaceMat.inverse();

    // Get the joint array.
    MArrayDataHandle influenceJointsHandle = dataBlock.inputArrayValue(influenceJointsAttr, &status);
    if(status != MS::kSuccess) return status;

    // Update the skeleton.
    vector<Transfo> bone_transforms;

    // The root joint is a dummy, and doesn't correspond with a Maya transform.
    bone_transforms.push_back(Transfo::identity());
    
    for(int i = 0; i < (int) influenceJointsHandle.elementCount(); ++i)
    {
        status = influenceJointsHandle.jumpToElement(i);
        if(status != MS::kSuccess) return status;

        // The world transform the joint has now:
        MDataHandle matrixHandle = influenceJointsHandle.inputValue(&status).child(influenceMatrixAttr);
        if(status != MS::kSuccess) return status;
        
        // The world transform the joint had at bind time:
        MDataHandle bindMatrixHandle = influenceJointsHandle.inputValue(&status).child(influenceBindMatrixAttr);
        if(status != MS::kSuccess) return status;

        // We need to get the change to the joint's transformation compared to when it was bound.
        // Maya gets this by multiplying the current worldMatrix against the joint's bindPreMatrix.
        // However, it's doing that in world space; we need it in object space.
        // XXX: There's probably a way to do this that doesn't require two matrix inversions.  It's
        // probably not worth caching, though.
        MMatrix bindPreMatrixWorldSpace = DagHelpers::readHandle<MMatrix>(bindMatrixHandle, &status); // original inverted world space transform
        MMatrix bindMatrixWorldSpace = bindPreMatrixWorldSpace.inverse();                // original (non-inverted) world space transform
        MMatrix bindMatrixObjectSpace = bindMatrixWorldSpace * worldToObjectSpaceMat;    // original object space transform
        MMatrix bindMatrixObjectSpaceInv = bindMatrixObjectSpace.inverse();              // original inverted object space transform

        MMatrix jointTransformWorldSpace = DagHelpers::readHandle<MMatrix>(matrixHandle, &status); // current world space transform

        MMatrix currentTransformObjectSpace = jointTransformWorldSpace * worldToObjectSpaceMat; // current object space transform
        MMatrix changeToTransform = bindMatrixObjectSpaceInv * currentTransformObjectSpace; // joint transform relative to bind pose in object space
        
        bone_transforms.push_back(DagHelpers::MMatrixToTransfo(changeToTransform));
    }

    // If we've been given fewer transformations than there are bones, set the missing ones to identity.
    if(cudaCtrl._skeleton.get_nb_joints() > bone_transforms.size())
        return MStatus::kFailure;
    bone_transforms.insert(bone_transforms.end(), cudaCtrl._skeleton.get_nb_joints() - bone_transforms.size(), Transfo::identity());

    // Update the skeleton transforms.
    cudaCtrl._skeleton.set_transforms(bone_transforms);

    // Update the vertex data.  We read all geometry, not just the set (if any) that we're being
    // applied to, so the algorithm can see the whole mesh.
    {
        // Get input.
        MArrayDataHandle inputArray = dataBlock.inputArrayValue(input, &status);
        if(status != MS::kSuccess) return status;

        // Get input[multiIndex].
        MDataHandle inputGeomData = DagHelpers::readArrayHandleLogicalIndex<MDataHandle>(inputArray, multiIndex, &status);
        if(status != MS::kSuccess) return status;

        // Get input[multiIndex].inputGeometry.
        MDataHandle inputGeomDataHandle = inputGeomData.child(inputGeom);

        // Load the vertex positions into cudaCtrl._anim_mesh.
        status = setGeometry(inputGeomDataHandle);
        if(status != MS::kSuccess) return status;
    }

    // Run the algorithm.  XXX: If we're being applied to a set, can we reduce the work we're doing to
    // just those vertices?
    cudaCtrl._anim_mesh->set_do_smoothing(true);
    cudaCtrl._anim_mesh->deform_mesh();

    vector<Point_cu> result_verts;
    cudaCtrl._anim_mesh->get_anim_vertices_aifo(result_verts);

    // Copy out the vertices that we were actually asked to process.
    for ( ; !geomIter.isDone(); geomIter.next()) {
        int vertex_index = geomIter.index();

        Point_cu v = result_verts[vertex_index];
        MPoint pt = MPoint(v.x, v.y, v.z);
        geomIter.setPosition(pt, MSpace::kObject);
    }

    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::load_skeleton(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // Load the skeleton from the node.
    // Note that we're still assuming that the current position of the
    // object is the same as bind time.  XXX: Is there anything we can do about that?
    // XXX Use skinCluster.geomMatrix (gm), and mirror it
    Loader::Abs_skeleton skeleton;
    status = createSkeleton(dataBlock, skeleton);
    if(status != MS::kSuccess) return status;

    // Load the skeleton into cudaCtrl.
    cudaCtrl._skeleton.load(skeleton);
    if(cudaCtrl._mesh != NULL && cudaCtrl.is_skeleton_loaded())
        cudaCtrl.load_animesh();

    return MStatus::kSuccess;
}

/*
 * Create an Abs_skeleton for the bind pose skeleton described by our attributes.
 */
MStatus ImplicitSkinDeformer::createSkeleton(MDataBlock &dataBlock, Loader::Abs_skeleton &skeleton)
{
    MStatus status = MStatus::kSuccess;

    // Get the geomMatrixAttr.
    MMatrix objectToWorldSpaceMat;
    status = DagHelpers::getMatrixPlug(thisMObject(), ImplicitSkinDeformer::geomMatrixAttr, objectToWorldSpaceMat);
    if(status != MS::kSuccess) return status;

    MMatrix worldToObjectSpaceMat = objectToWorldSpaceMat.inverse();

    // Create a root bone.
    {
        // The root bone na
        Loader::Abs_bone bone;
        bone._name = "";

        // Create this joint at the origin in world space, and transform it to object space like the
        // ones we create below.  (Multiplying by identity doesn't do anything; it's only written this
        // way for clarity.)
        MMatrix jointObjectMat = MMatrix::identity * worldToObjectSpaceMat;
        bone._frame = DagHelpers::MMatrixToTransfo(jointObjectMat);

        skeleton._bones.push_back(bone);
        skeleton._sons.push_back(std::vector<int>());
        skeleton._parents.push_back(-1);
        skeleton._root = 0;
    }

    // Create the bones.
    MArrayDataHandle influenceJointsHandle = dataBlock.inputArrayValue(ImplicitSkinDeformer::influenceJointsAttr, &status);
    if(status != MS::kSuccess) return status;

    for(int i = 0; i < (int) influenceJointsHandle.elementCount(); ++i)
    {
        status = influenceJointsHandle.jumpToElement(i);
        if(status != MS::kSuccess) return status;

        int boneIdx = (int) skeleton._bones.size(); 

        // Get bind positions from the bindPreMatrix.
        MMatrix jointWorldMat;

        {
            MDataHandle influenceBindMatrixHandle = influenceJointsHandle.inputValue(&status).child(ImplicitSkinDeformer::influenceBindMatrixAttr);
            if(status != MS::kSuccess) return status;

            MMatrix influenceBindMatrix = DagHelpers::readHandle<MMatrix>(influenceBindMatrixHandle, &status);
            if(status != MS::kSuccess) return status;
            
            // This is the inverse matrix.  We want the forwards matrix, so un-invert it.
            jointWorldMat = influenceBindMatrix.inverse();
        }

        MMatrix jointObjectMat = jointWorldMat * worldToObjectSpaceMat;

        // Fill in the bone.  We don't need to calculate _length, since compute_bone_lengths will
        // do it below.
        Loader::Abs_bone bone;
        bone._frame = DagHelpers::MMatrixToTransfo(jointObjectMat);
        skeleton._bones.push_back(bone);


        // Read this joint's parent joint index.
        MDataHandle parentJointHandle = influenceJointsHandle.inputValue(&status).child(ImplicitSkinDeformer::parentJointAttr);
        if(status != MS::kSuccess) return status;

        int parentIdx = DagHelpers::readHandle<int>(parentJointHandle, &status);
        if(status != MS::kSuccess) return status;

        // parentIndexes don't include the root joint 0, so add 1.  A parentIndex of -1
        // means the root joint, so we can just add 1 in that case too to get 0.
        parentIdx++;

        skeleton._parents.push_back(parentIdx);

        // Add this bone as a child of its parent.
        skeleton._sons.push_back(std::vector<int>());
        if(parentIdx != -1)
            skeleton._sons[parentIdx].push_back(boneIdx);
    }

    skeleton.compute_bone_lengths();
    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::save_sampleset(const SampleSet::SampleSet &samples)
{
    MStatus status = MStatus::kSuccess;

    MPlug jointArrayPlug(thisMObject(), ImplicitSkinDeformer::influenceJointsAttr);
    
    // Skip the dummy root joint.
    for(int i = 1; i < (int) samples._samples.size(); ++i)
    {
        const SampleSet::InputSample &inputSample = samples._samples[i];

        MPlug jointPlug = jointArrayPlug.elementByLogicalIndex(i-1, &status);
        if(status != MS::kSuccess) return status;

        // Save the junction radius.
        {
            MPlug junctionRadiusPlug = jointPlug.child(ImplicitSkinDeformer::junctionRadiusAttr, &status);
            if(status != MS::kSuccess) return status;

            status = junctionRadiusPlug.setValue(inputSample._junction_radius);
            if(status != MS::kSuccess) return status;
        }

        // XXX caps, jcap/pcap flags (only one or the other?  if caps are editable, should they
        // be changed to regular samples?)

        // Save the samples.
        MPlug samplePointPlug = jointPlug.child(ImplicitSkinDeformer::samplePointAttr, &status);
        if(status != MS::kSuccess) return status;

        MPlug sampleNormalPlug = jointPlug.child(ImplicitSkinDeformer::sampleNormalAttr, &status);
        if(status != MS::kSuccess) return status;
        for(int sampleIdx = 0; sampleIdx < inputSample._sample_list.nodes.size(); ++sampleIdx)
        {
            MPlug samplePlug = samplePointPlug.elementByLogicalIndex(sampleIdx, &status);
            if(status != MS::kSuccess) return status;

            status = DagHelpers::setPlugValue(samplePlug,
                inputSample._sample_list.nodes[sampleIdx].x,
                inputSample._sample_list.nodes[sampleIdx].y,
                inputSample._sample_list.nodes[sampleIdx].z);
            if(status != MStatus::kSuccess) return status;

            MPlug normalPlug = sampleNormalPlug.elementByLogicalIndex(sampleIdx, &status);
            if(status != MS::kSuccess) return status;

            status = DagHelpers::setPlugValue(normalPlug,
                inputSample._sample_list.n_nodes[sampleIdx].x,
                inputSample._sample_list.n_nodes[sampleIdx].y,
                inputSample._sample_list.n_nodes[sampleIdx].z);
            if(status != MStatus::kSuccess) return status;
        }
    }

    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::load_sampleset(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // If the mesh isn't loaded yet, don't do anything.
    if(cudaCtrl._anim_mesh == NULL)
        return MStatus::kSuccess;

    MArrayDataHandle influenceJointsHandle = dataBlock.inputArrayValue(ImplicitSkinDeformer::influenceJointsAttr, &status);
    if(status != MS::kSuccess) return status;

    // Create a new SampleSet, and load its values from the node.
    SampleSet::SampleSet samples(cudaCtrl._anim_mesh->get_skel()->nb_joints());

    // Skip the dummy root joint.
    for(int i = 0; i < (int) influenceJointsHandle.elementCount(); ++i)
    {
        SampleSet::InputSample &inputSample = samples._samples[i+1];

        status = influenceJointsHandle.jumpToElement(i);
        if(status != MS::kSuccess) return status;

        MDataHandle junctionRadiusHandle = influenceJointsHandle.inputValue(&status).child(ImplicitSkinDeformer::junctionRadiusAttr);
        if(status != MS::kSuccess) return status;

        inputSample._junction_radius = DagHelpers::readHandle<float>(junctionRadiusHandle, &status);
        if(status != MS::kSuccess) return status;

        // Load the samples.
        MArrayDataHandle samplePointHandle = influenceJointsHandle.inputValue(&status).child(ImplicitSkinDeformer::samplePointAttr);
        if(status != MS::kSuccess) return status;

        MArrayDataHandle sampleNormalHandle = influenceJointsHandle.inputValue(&status).child(ImplicitSkinDeformer::sampleNormalAttr);
        if(status != MS::kSuccess) return status;

        if(samplePointHandle.elementCount() != sampleNormalHandle.elementCount())
            return MStatus::kFailure;

        for(int sampleIdx = 0; sampleIdx < (int) samplePointHandle.elementCount(); ++sampleIdx)
        {
            status = samplePointHandle.jumpToElement(sampleIdx);
            if(status != MS::kSuccess) return status;

            status = sampleNormalHandle.jumpToElement(sampleIdx);
            if(status != MS::kSuccess) return status;

            DagHelpers::simpleFloat3 samplePoint = DagHelpers::readArrayHandle<DagHelpers::simpleFloat3>(samplePointHandle, &status);
            if(status != MS::kSuccess) return status;

            DagHelpers::simpleFloat3 sampleNormal = DagHelpers::readArrayHandle<DagHelpers::simpleFloat3>(sampleNormalHandle, &status);
            if(status != MS::kSuccess) return status;

            inputSample._sample_list.nodes.push_back(Vec3_cu(samplePoint.x, samplePoint.y, samplePoint.z));

            inputSample._sample_list.n_nodes.push_back(Vec3_cu(sampleNormal.x, sampleNormal.y, sampleNormal.z));
        }
    }

    // Load the SampleSet into _anim_mesh.
    cudaCtrl._anim_mesh->set_sampleset(samples);

    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::load_mesh(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // Load the geometry.
    MDataHandle geomHandle = dataBlock.inputValue(ImplicitSkinDeformer::unskinnedGeomAttr, &status);
    if(status != MS::kSuccess) return status;

    MObject geom = geomHandle.asMesh();
    if(!geom.hasFn(MFn::kMesh)) {
        // XXX: only meshes are supported
        return MStatus::kFailure;
    }

    // Load the input mesh from the unskinned geometry.
    Loader::Abs_mesh loaderMesh;
    status = MayaData::load_mesh(geom, loaderMesh);
    if(status != MS::kSuccess) return status;

    // Abs_mesh is a simple representation that doesn't touch CUDA.  Load it into Mesh.
    Mesh *mesh = new Mesh(loaderMesh);

    // Hand the Mesh to Cuda_ctrl.
    cudaCtrl.load_mesh(mesh);

    // XXX: This will wipe out the loaded bones and require a skeleton/sampleset reload
    if(cudaCtrl._mesh != NULL && cudaCtrl.is_skeleton_loaded())
        cudaCtrl.load_animesh();

    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::setGeometry(MDataHandle &inputGeomDataHandle)
{
    MItGeometry allGeomIter(inputGeomDataHandle, true);

    MPointArray points;
    allGeomIter.allPositions(points, MSpace::kObject);

    // If the geometry doesn't have the same number of vertices, we can't use it.  This can be
    // caused by a deformer like deleteVertices being added between us and the skinCluster, and the
    // user should bake it (delete non-deformer history).
    // XXX: Is there a way we can tell the user about this?
    // XXX: Will the algorithm allow us to support this, if we give it a whole new mesh with similar
    // topology and call update_base_potential?
    if(points.length() != cudaCtrl._anim_mesh->get_nb_vertices())
        return MStatus::kSuccess;

    // Set the deformed vertex data.
    vector<Vec3_cu> input_verts;
    input_verts.reserve(points.length());
    for(int i = 0; i < (int) points.length(); ++i)
        input_verts.push_back(Vec3_cu((float) points[i].x, (float) points[i].y, (float) points[i].z));
        
    cudaCtrl._anim_mesh->copy_vertices(input_verts);

    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::set_bind_pose()
{
    MStatus status = MStatus::kSuccess;

    // Reset the relative joint transformations to identity, eg. no change after setup.  deform() will reload the
    // current transforms the next time it needs them.
    vector<Transfo> bone_transforms(cudaCtrl._skeleton.get_nb_joints(), Transfo::identity());
    cudaCtrl._skeleton.set_transforms(bone_transforms);

    // Load the unskinned geometry, which matches with the identity transforms.
    MPlug unskinnedGeomPlug(thisMObject(), unskinnedGeomAttr);
    MDataHandle inputGeomDataHandle;
    status = unskinnedGeomPlug.getValue(inputGeomDataHandle);
    if(status != MS::kSuccess) return status;

    status = setGeometry(inputGeomDataHandle);
    if(status != MS::kSuccess) return status;

    return MStatus::kSuccess;
}

// Update the base potential for the current mesh and samples.  This requires loading the unskinned geometry.
MStatus ImplicitSkinDeformer::load_base_potential(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // Make sure our dependencies are up to date.
    dataBlock.inputValue(ImplicitSkinDeformer::sampleSetUpdateAttr, &status);
    if(status != MS::kSuccess) return status;
    dataBlock.inputValue(ImplicitSkinDeformer::meshUpdateAttr, &status);
    if(status != MS::kSuccess) return status;
    dataBlock.inputValue(ImplicitSkinDeformer::skeletonUpdateAttr, &status);
    if(status != MS::kSuccess) return status;

    // If we don't have a mesh yet, don't do anything.
    if(cudaCtrl._anim_mesh == NULL)
        return MStatus::kSuccess;

    set_bind_pose();

    // Update base potential while in bind pose.
    cudaCtrl._anim_mesh->update_base_potential();

    return MStatus::kSuccess;
}

class ImplicitCommand : public MPxCommand
{
public:
    virtual ~ImplicitCommand() { }
    MStatus doIt( const MArgList& );
//    MStatus redoIt();
//    MStatus undoIt();

    MStatus init(MString nodeName);
    MStatus sampleAll(MString nodeName); // runs all sampling

    ImplicitSkinDeformer *getDeformerByName(MString nodeName, MStatus *status);

    bool isUndoable() const { return false; }
    static void *creator() { return new ImplicitCommand(); }

private:
    MStatus getOnePlugByName(MString nodeName, MPlug &plug);
};

MStatus ImplicitCommand::getOnePlugByName(MString nodeName, MPlug &plug)
{
    MSelectionList slist;
    MStatus status = slist.add(nodeName);
    if(status != MS::kSuccess) return status;

    int matches = slist.length(&status);
    if(status != MS::kSuccess) return status;

    if(matches > 1)
    {
        displayError("Multiple nodes found: " + nodeName);
        return MS::kFailure;
    }

    MPlug implicitPlug;
    return slist.getPlug(0, plug);
}

MStatus ImplicitSkinDeformer::sample_all_joints()
{
    MStatus status = MStatus::kSuccess;

    // Force skeletonUpdateAttr to be updated.
    loadDependency(thisMObject(), ImplicitSkinDeformer::skeletonUpdateAttr, &status);
    loadDependency(thisMObject(), ImplicitSkinDeformer::meshUpdateAttr, &status);
    if(status != MS::kSuccess) return status;

    // If we don't have a mesh yet, don't do anything.
    if(cudaCtrl._anim_mesh == NULL)
        return MStatus::kSuccess;

    set_bind_pose();

    // Run the initial sampling.  Skip bone 0, which is a dummy parent bone.
    SampleSet::SampleSet samples(cudaCtrl._anim_mesh->get_skel()->nb_joints());

    // Get the default junction radius.
    vector<float> junction_radius;
    cudaCtrl._anim_mesh->get_default_junction_radius(junction_radius);

    for(int bone_id = 1; bone_id < cudaCtrl._anim_mesh->get_skel()->nb_joints(); ++bone_id)
    {
        samples._samples[bone_id]._junction_radius = junction_radius[bone_id];
        
        if(true)
        {
            samples.choose_hrbf_samples_poisson
                    (*cudaCtrl._anim_mesh->_animesh,
                     bone_id,
                     // Set a distance threshold from sample to the joints to choose them.
                     -0.02f, // dSpinB_max_dist_joint->value(),
                     -0.02f, // dSpinB_max_dist_parent->value(),
                     0, // dSpinB_min_dist_samples->value(),
                     // Minimal number of samples.  (this value is used only whe the value min dist is zero)
                     50, // spinB_nb_samples_psd->value(), 20-1000

                     // We choose a sample if: max fold > (vertex orthogonal dir to the bone) dot (vertex normal)
                     0); // dSpinB_max_fold->value()
        } else {
            samples.choose_hrbf_samples_ad_hoc
                    (*cudaCtrl._anim_mesh->_animesh,
                     bone_id,
                     -0.02f, // dSpinB_max_dist_joint->value(),
                     -0.02f, // dSpinB_max_dist_parent->value(),
                     0, // dSpinB_min_dist_samples->value(), Minimal distance between two HRBF sample
                     0); // dSpinB_max_fold->value()
        }
    }

    // Save the new SampleSet.
    return save_sampleset(samples);
}

ImplicitSkinDeformer *ImplicitCommand::getDeformerByName(MString nodeName, MStatus *status)
{
    // Get the MPlug for the selected node.
    MPlug implicitPlug;
    *status = getOnePlugByName(nodeName, implicitPlug);
    if(*status != MS::kSuccess) return NULL;

    MFnDependencyNode plugDep(implicitPlug.node(), status);
    if(*status != MS::kSuccess) return NULL;

    // Verify that this is one of our nodes.
    MTypeId type = plugDep.typeId(status);
    if(*status != MS::kSuccess) return NULL;

    if(type != ImplicitSkinDeformer::id)
    {
        displayError("Node not an implicitDeformer: " + nodeName);
        *status = MStatus::kFailure;;
        return NULL;
    }

    ImplicitSkinDeformer *deformer = (ImplicitSkinDeformer *) plugDep.userNode(status);
    if(*status != MS::kSuccess) return NULL;
    return deformer;
}

MStatus ImplicitCommand::init(MString nodeName)
{
    MStatus status;
    ImplicitSkinDeformer *deformer = getDeformerByName(nodeName, &status);
    if(status != MS::kSuccess) return status;

    // Create an MFnGeometryFilter on the ImplicitSkinDeformer.
    MFnGeometryFilter implicitGeometryFilter(deformer->thisMObject(), &status);
    if(status != MS::kSuccess) return status;

    // Ask the MFnGeometryFilter for the MDagPath of the output, and pop to get to the transform
    // node.
    MDagPath implicitGeometryOutputDagPath;
    implicitGeometryFilter.getPathAtIndex(0, implicitGeometryOutputDagPath);
    implicitGeometryOutputDagPath.pop();

    {
        // Get the inverse transformation, which is the transformation to go from world space to
        // object space.
        MFnTransform transformNode(implicitGeometryOutputDagPath.node());
        MMatrix objectToWorldSpaceMat = transformNode.transformationMatrix(&status);
        MMatrix worldToObjectSpaceMat = objectToWorldSpaceMat.inverse();

        // Store objectToWorldSpaceMat on geomMatrixAttr.
        status = DagHelpers::setMatrixPlug(deformer->thisMObject(), ImplicitSkinDeformer::geomMatrixAttr, objectToWorldSpaceMat);
        if(status != MS::kSuccess) return status;
    }

    // Find the skinCluster deformer node above the deformer.
    MObject skinClusterNode;
    status = DagHelpers::findAncestorDeformer(deformer->thisMObject(), MFn::kSkinClusterFilter, skinClusterNode);
    if(status != MS::kSuccess)
    {
        printf("Couldn't find a skinCluster deformer.  Is the node skinned?\n");
        return status;
    }

    // For each influence going into the skinCluster's .matrix array, connect it to our .matrix array
    // as well.
    MPlug jointArrayPlug(deformer->thisMObject(), ImplicitSkinDeformer::influenceJointsAttr);

    {
        MFnDependencyNode skinClusterDep(skinClusterNode);
        const MObject skinClusterMatrixObject = skinClusterDep.attribute("matrix", &status);
        if(status != MS::kSuccess) return status;

        MPlug skinClusterMatrixArray(skinClusterNode, skinClusterMatrixObject);
        skinClusterMatrixArray.evaluateNumElements(&status);
        if(status != MS::kSuccess) return status;

        MDGModifier dgModifier;

        for(int i = 0; i < (int) skinClusterMatrixArray.numElements(); ++i)
        {
            MPlug skinClusterMatrixElementPlug = skinClusterMatrixArray.elementByPhysicalIndex(i, &status);
            if(status != MS::kSuccess) return status;

            // XXX: test this if a skinCluster has deleted influences
            MPlugArray plugArray;
            skinClusterMatrixElementPlug.connectedTo(plugArray, true /* asDst */, false /* asSrc */, &status);
            if(status != MS::kSuccess) return status;

            if(plugArray.length() == 0)
                continue;

            // The joint's worldMatrix plug, which is connected to the skinCluster's matrix[n] plug.
            MPlug connectionPlug = plugArray[0];


            // Get the logical index on the skinCluster.matrix array, which we'll mirror.
            int elementLogicalIndex = skinClusterMatrixElementPlug.logicalIndex(&status);
            if(status != MS::kSuccess) return status;

            MPlug jointPlug = jointArrayPlug.elementByLogicalIndex(elementLogicalIndex, &status);
            if(status != MS::kSuccess) return status;

            MPlug matrixElementPlug = jointPlug.child(ImplicitSkinDeformer::influenceMatrixAttr, &status);
            if(status != MS::kSuccess) return status;

            status = dgModifier.connect(connectionPlug, matrixElementPlug);
            if(status != MS::kSuccess) return status;
        }

        dgModifier.doIt();
    }

    {
        vector<int> parentIndexes;
        status = MayaData::loadSkeletonHierarchyFromSkinCluster(skinClusterNode, parentIndexes);
        if(status != MS::kSuccess) return status;

        // Copy bindPreMatrix from the skinCluster to influenceBindMatrix.  This stores the transform for
        // each influence at the time setup was done.
        MFnDependencyNode skinClusterDep(skinClusterNode);
        const MObject bindPreMatrixObject = skinClusterDep.attribute("bindPreMatrix", &status);
        if(status != MS::kSuccess) return status;

        MPlug bindPreMatrixArray(skinClusterNode, bindPreMatrixObject);
        bindPreMatrixArray.evaluateNumElements(&status);
        if(status != MS::kSuccess) return status;

        for(int i = 0; i < (int) bindPreMatrixArray.numElements(); ++i)
        {
            MPlug bindPreMatrix = bindPreMatrixArray.elementByPhysicalIndex(i, &status);
            if(status != MS::kSuccess) return status;

            int elementLogicalIndex = bindPreMatrix.logicalIndex(&status);
            if(status != MS::kSuccess) return status;

            MPlug jointPlug = jointArrayPlug.elementByLogicalIndex(elementLogicalIndex, &status);
            if(status != MS::kSuccess) return status;

            MPlug item = jointPlug.child(ImplicitSkinDeformer::influenceBindMatrixAttr, &status);
            if(status != MS::kSuccess) return status;

            MMatrix bindPreMatrixWorldSpace = DagHelpers::getMatrixFromPlug(bindPreMatrix, &status);
            if(status != MS::kSuccess) return status;

            status = DagHelpers::setPlug(item, bindPreMatrixWorldSpace);
            if(status != MS::kSuccess) return status;

            item = jointPlug.child(ImplicitSkinDeformer::parentJointAttr, &status);
            if(status != MS::kSuccess) return status;

            status = item.setValue(parentIndexes[i]);
            if(status != MS::kSuccess) return status;
        }
    }

    // Get the inputGeometry going into the skinCluster.  This is the mesh before skinning, which
    // we'll use to do initial calculations.  The bind-time positions of the joints we got above
    // should correspond with the pre-skinned geometry.

    MPlug unskinnedGeomPlug(deformer->thisMObject(), ImplicitSkinDeformer::unskinnedGeomAttr);
    {
        // Get the inputGeometry going into the skinCluster.  This is the mesh before skinning, which
        // we'll use to do initial calculations.  The bind-time positions of the joints we got above
        // should correspond with the pre-skinned geometry.
        MPlug skinInputGeometryPlug;
        status = DagHelpers::getInputGeometryForSkinClusterPlug(skinClusterNode, skinInputGeometryPlug);
        if(status != MS::kSuccess) return status;

        // Connect the input geometry to unskinnedGeomPlug.  This way, we can always get access to
        // the base geometry without having to dig through the DG.
        MDGModifier dgModifier;
        status = dgModifier.connect(skinInputGeometryPlug, unskinnedGeomPlug);
        if(status != MS::kSuccess) return status;
        dgModifier.doIt();
    }

    return MStatus::kSuccess;
}

MStatus ImplicitCommand::sampleAll(MString nodeName)
{
    MStatus status;
    ImplicitSkinDeformer *deformer = getDeformerByName(nodeName, &status);
    if(status != MS::kSuccess) return status;

    return deformer->sample_all_joints();
}

MStatus ImplicitCommand::doIt(const MArgList &args)
{
    MStatus status;
    for(int i = 0; i < (int) args.length(); ++i)
    {
        if(args.asString(i, &status) == MString("-init") && MS::kSuccess == status)
        {
            ++i;
            MString nodeName = args.asString(i, &status);
            if(status != MS::kSuccess) return status;

            status = init(nodeName);

            if(status != MS::kSuccess) {
                displayError(status.errorString());
                return status;
            }
        }
        else if(args.asString(i, &status) == MString("-sampleAll") && MS::kSuccess == status)
        {
            ++i;
            MString nodeName = args.asString(i, &status);
            if(status != MS::kSuccess) return status;

            status = sampleAll(nodeName);

            if(status != MS::kSuccess) {
                displayError(status.errorString());
                return status;
            }
        }
    }
    return MS::kSuccess;
}



MStatus initializePlugin(MObject obj)
{
    MStatus status;

    std::vector<Blending_env::Op_t> op;
    op.push_back( Blending_env::B_D  );
    op.push_back( Blending_env::U_OH );
    op.push_back( Blending_env::C_D  );

    Cuda_ctrl::cuda_start(op);

    // XXX "HACK: Because blending_env initialize to elbow too ..." What?
    IBL::Ctrl_setup shape = IBL::Shape::elbow();

    MFnPlugin plugin(obj, "", "1.0", "Any");

    status = plugin.registerNode("implicitSkin", ImplicitSkinDeformer::id, ImplicitSkinDeformer::creator, ImplicitSkinDeformer::initialize, MPxNode::kDeformerNode);
    if(status != MS::kSuccess) return status;

    status = plugin.registerCommand("implicitSkin", ImplicitCommand::creator);
    if(status != MS::kSuccess) return status;

    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;

    Cuda_ctrl::cleanup();

    MFnPlugin plugin(obj);

    status = plugin.deregisterNode(ImplicitSkinDeformer::id);
    if(status != MS::kSuccess) return status;

    status = plugin.deregisterCommand("implicitSkin");
    if(status != MS::kSuccess) return status;

    return MS::kSuccess;
}
