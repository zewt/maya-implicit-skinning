// Don't include CUDA headers in this file.  CUDA and Maya headers are incompatible due to
// namespace conflicts.
#define NO_CUDA

#include "plugin.hpp"

#include <maya/MGlobal.h> 
#include <maya/MPxCommand.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>

#include <maya/MItSelectionList.h>
#include <maya/MArrayDataBuilder.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixData.h>

#include <maya/MFnDagNode.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnTransform.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnPluginData.h>
#include <maya/MFnSkinCluster.h>

#include <maya/MArgList.h>
#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MFnMesh.h>

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MArrayDataHandle.h>

#include <maya/MDagModifier.h>

#include "maya/maya_helpers.hpp"
#include "maya/maya_data.hpp"
#include "utils/misc_utils.hpp"
#include "utils/std_utils.hpp"

#include "skeleton.hpp"
#include "sample_set.hpp"
#include "cuda_ctrl.hpp"
#include "hrbf_env.hpp"
#include "vert_to_bone_info.hpp"

#include <string.h>
#include <math.h>
#include <assert.h>
#include <algorithm>
#include <map>
using namespace std;

class ImplicitCommand : public MPxCommand
{
public:
    virtual ~ImplicitCommand() { }
    MStatus doIt( const MArgList& );
//    MStatus redoIt();
//    MStatus undoIt();

    void init(MString nodeName);
    void calculate_base_potential(MString deformerName);

    ImplicitDeformer *getDeformerByName(MString nodeName);

    bool isUndoable() const { return false; }
    static void *creator() { return new ImplicitCommand(); }
    void test(MString nodeName);

private:
    MPlug getOnePlugByName(MString nodeName);
};

MPlug ImplicitCommand::getOnePlugByName(MString nodeName)
{
    MSelectionList slist;
    MStatus status = slist.add(nodeName);
    if(status != MS::kSuccess)
        throw runtime_error(string("Node \"") + nodeName.asChar() + "\" not found.");

    int matches = slist.length(&status); merr("slist.length");

    if(matches > 1)
        throw runtime_error(string("Multiple nodes found: ") + nodeName.asChar());

    MPlug plug;
    status = slist.getPlug(0, plug); merr("slist.getPlug");
    return plug;
}

ImplicitDeformer *ImplicitCommand::getDeformerByName(MString nodeName)
{
    MStatus status = MStatus::kSuccess;

    // Get the MPlug for the selected node.
    MPlug implicitPlug = getOnePlugByName(nodeName);

    ImplicitDeformer *result = DagHelpers::GetInterfaceFromNode<ImplicitDeformer>(implicitPlug.node(), &status);
    merr("DagHelpers::GetInterfaceFromNode<ImplicitDeformer>");
    return result;
}

void ImplicitCommand::calculate_base_potential(MString deformerName)
{
    MStatus status = MStatus::kSuccess;

    ImplicitDeformer *deformer = getDeformerByName(deformerName);
    status = deformer->calculate_base_potential(); merr("calculate_base_potential");
}

// Create a shape node of a custom type, and return its interface.
//
// The shape name will be suffixed with "Shape", and the given name will be assigned to
// the surrounding transform.  If transformNodeOut is non-NULL, it will be set to the
// transform node.
template<typename T>
T *createShape(MString name, MObject *transformNodeOut)
{
    MStatus status = MStatus::kSuccess;

    // Create the shape node.  We'll actually be given the transform node created around it.
    MFnDependencyNode depNode;
    MObject transformNode = depNode.create(T::id, name + "Shape", &status);
    if(transformNodeOut != NULL)
        *transformNodeOut = transformNode;

    // Name the transform node that was created above our shape.
    MFnDependencyNode transformDepNode(transformNode, &status); merr("transformDepNode");

    transformDepNode.setName(name, &status); merr("setName");
    if(status != MS::kSuccess) return NULL;

    // Get our actual node from the transform.
    MObject shapeNode = DagHelpers::getShapeUnderNode(transformNode, &status); merr("getShapeUnderNode");
    if(status != MS::kSuccess) return NULL;

    // Pull out the ImplicitSurface interface from the node.
    T *result = DagHelpers::GetInterfaceFromNode<T>(shapeNode, &status); merr("GetInterfaceFromNode");
    return result;
}

namespace {
    struct AbsSkeletonBone {
        Transfo bone;
        int parent;
    };
    struct Abs_skeleton {
        /// List of bones
        std::vector<Transfo> _bones;
        /// _parents[bone_id] == parent_bone_id
        std::vector<int> _parents;
        std::vector<MDagPath> dagPaths;

        // The physical index for each value in _bones.  Bones that don't actually have an
        // influence object and don't really exist will have no entry.
        map<int,int> logicalIndexToPhysicalIndex;

        void load(MObject skinClusterNode)
        {
            MStatus status = MStatus::kSuccess;

            map<int,MDagPath> logicalIndexToInfluenceObjects;
            status = DagHelpers::getInfluenceObjectsForSkinCluster(skinClusterNode, logicalIndexToInfluenceObjects, logicalIndexToPhysicalIndex);
            merr("getInfluenceObjectsForSkinCluster");

            map<int,int> logicalIndexToParentIdx;
            MayaData::loadSkeletonHierarchyFromSkinCluster(logicalIndexToInfluenceObjects, logicalIndexToParentIdx);

            // Order the indexes parent nodes first.
            vector<int> hierarchyOrder;
            if(!MiscUtils::getHierarchyOrder(logicalIndexToParentIdx, hierarchyOrder))
                throw runtime_error("The input hierarchy contains cycles.");

            for(int logicalIndex: hierarchyOrder)
            {
                const MDagPath &influenceObjectPath = logicalIndexToInfluenceObjects.at(logicalIndex);

                MMatrix jointWorldMat = influenceObjectPath.inclusiveMatrix(&status); merr("influenceObjectPath.inclusiveMatrix");

                // Make space for the item, if needed.
                int size = max((int) _parents.size(), logicalIndex+1);
                _parents.resize(size, -1);
                _bones.resize(size);
                dagPaths.resize(size);

                // Add the bone.
                _bones[logicalIndex] = DagHelpers::MMatrixToTransfo(jointWorldMat);
                _parents[logicalIndex] = logicalIndexToParentIdx[logicalIndex];
                dagPaths[logicalIndex] = influenceObjectPath;
            }
        }
    };

    struct BoneItem
    {
        BoneItem(std::shared_ptr<Bone> bone_) {
            bone = bone_;
            parent = -1;
        }

        std::shared_ptr<Bone> bone;

        // The logical index of this bone in the skinCluster's influences.
        int influence_logical_index;

        // The physical index of this bone in the skinCluster's influences, or -1 if this
        // bone has no entry.
        int physicalIndex;

        Bone::Id parent;

        // The DAG path to the influence this surface was created from.
        MDagPath dagPath;

        // The DAG path to the node that this surface should be parented under.
        // This may be different from the path to "parent".  When empty surfaces are
        // removed, the parent is updated to point to the next parent up the hierarchy,
        // but the Maya parent influence always remains the same, since the surface is
        // still affected by the same joint.
        MDagPath parentInfluence;

    private:
        BoneItem &operator=(BoneItem &rhs) { return *this; }
        BoneItem(const BoneItem &rhs) { }
    };

    void loadBones(const Abs_skeleton &skel, std::map<Bone::Id, BoneItem> &bones)
    {
        std::map<int,Bone::Id> loaderIdxToBoneId;
        std::map<Bone::Id,int> boneIdToLoaderIdx;

        // Create the Bones.
        for(int idx = 0; idx < (int) skel._bones.size(); idx++)
        {
            std::shared_ptr<Bone> bone(new Bone());

            BoneItem &item = bones.emplace(bone->get_bone_id(), bone).first->second;
            item.dagPath = skel.dagPaths[idx];
            item.influence_logical_index = idx;
            item.physicalIndex = Std_utils::get(skel.logicalIndexToPhysicalIndex, idx, -1);

            loaderIdxToBoneId[idx] = item.bone->get_bone_id();
            boneIdToLoaderIdx[item.bone->get_bone_id()] = idx;
        }

        // Set up each BoneItem's parent.
        for(auto &it: bones)
        {
            BoneItem &item = it.second;
            Bone::Id bid = item.bone->get_bone_id();

            // Set up item.parent.
            int loader_idx = boneIdToLoaderIdx.at(bid);
            int loader_parent_idx = skel._parents[loader_idx];
            if(loader_parent_idx != -1) {
                item.parent = loaderIdxToBoneId.at(loader_parent_idx);

                // Store the DAG path that this node's parent is for.  This is the node that this
                // surface will be parented under for transforms.
                if(item.parent != -1)
                {
                    const BoneItem &parentItem = bones.at(item.parent);
                    item.parentInfluence = skel.dagPaths[parentItem.influence_logical_index];
                }
            }
        }

        // Set up each bone.
        for(auto &it: bones)
        {
            // Set up _bone.
            Bone::Id bone_id = it.first;
            BoneItem &item = it.second;
            int bone_loader_idx = boneIdToLoaderIdx.at(bone_id);

            Bone::Id parent_bone_id = item.parent;
            Vec3_cu org = Transfo::identity().get_translation();
            if(parent_bone_id != -1) {
                int parent_bone_loader_idx = boneIdToLoaderIdx.at(parent_bone_id);
                org = skel._bones[parent_bone_loader_idx].get_translation();
            }

            Vec3_cu end = skel._bones[bone_loader_idx].get_translation();
            Vec3_cu dir = end.to_point() - org.to_point();
            float length = dir.norm();

            Bone_cu initial_position = Bone_cu(org.to_point(), dir, length);
            item.bone->set_length(initial_position._length);
            item.bone->set_orientation(initial_position.org(), initial_position.dir());

            // If any bones lie on the same position as their parent, they'll have a zero length and
            // an undefined orientation.  Set them to a small length and a default orientation.
            if(item.bone->length() < 0.000001f)
            {
                item.bone->set_length(0.000001f);
                item.bone->_dir = Vec3_cu(1,0,0);
            }
        }
    }

    // Return a Mesh loaded from the output of the given skin cluster.
    std::unique_ptr<Mesh> createMeshFromSkinClusterOutput(MObject skinClusterNode)
    {
        MStatus status = MStatus::kSuccess;
        MFnSkinCluster skinCluster(skinClusterNode, &status); merr("skinCluster()");

        MDagPath skinClusterOutputPath;
        status = skinCluster.getPathAtIndex(0, skinClusterOutputPath); merr("skinCluster.getPathAtIndex(0)");
    
        MObject skinClusterOutputShape = skinClusterOutputPath.node();
        if(!skinClusterOutputShape.hasFn(MFn::kMesh))
            throw runtime_error("skinClusterOutputShape doesn't have MFn::kMesh.  Only meshes are supported.");

        // Load the input mesh from the skinCluster's output.
        Loader::Abs_mesh loaderMesh;
        MMatrix objectToWorldMatrix = skinClusterOutputPath.inclusiveMatrix(&status);
        status = MayaData::load_mesh(skinClusterOutputShape, loaderMesh, objectToWorldMatrix); merr("MayaData::load_mesh");

        // Abs_mesh is a simple representation that doesn't touch CUDA.  Load it into Mesh.
        std::unique_ptr<Mesh> mesh(new Mesh(loaderMesh));
        mesh->check_integrity();
        return mesh;
    }

    void clusterVerticesToBones(MObject skinClusterNode, const Mesh *mesh, const std::map<Bone::Id, BoneItem> &boneItems, std::vector< std::vector<Bone::Id> > &bonesPerVertex)
    {
        // Retrieve skin weights.  We'll use these to cluster vertices to surfaces.
        vector<vector<double> > weightsPerIndex;
        MStatus status = DagHelpers::getWeightsForAllVertices(skinClusterNode, weightsPerIndex); merr("DagHelpers::getWeightsForAllVertices");

        // Make a list of bones that we want each vertex to be a part of.  A vertex can be in
        // more than one bone.
        bonesPerVertex.reserve(weightsPerIndex.size());
        for(int vertIdx = 0; vertIdx < (int) weightsPerIndex.size(); ++vertIdx)
        {
            vector<double> &weights = weightsPerIndex[vertIdx];
            bonesPerVertex.emplace_back();
            std::vector<int> &bones = bonesPerVertex.back();

            Point_cu currentVertex = mesh->get_vertex(vertIdx).to_point();

            // Look at each bone, and decide if we want this vertex to be included in it.
            int closestBoneId = -1;
            double closestBoneDistance = 999999;
            for(auto &it: boneItems)
            {
                Bone::Id boneId = it.first;
                const BoneItem &boneItem = it.second;

                // We create a surface going from each joint to its parent, using the vertices
                // that are influenced by the parent.  Root joints don't create surfaces, so skip
                // them.
                Bone::Id parentBoneId = boneItem.parent;
                if(parentBoneId == -1)
                    continue;

                // If the parent joint has no physical index, then it doesn't influence any
                // vertices.
                const BoneItem &parentBoneItem = boneItems.at(parentBoneId);
                if(parentBoneItem.physicalIndex == -1)
                    continue;

                // Ignore very low weights.
                double weight = weights[parentBoneItem.physicalIndex];
                double threshold = 0.05;
                if(weight < threshold)
                    continue;

                float distanceFromBone = boneItem.bone->dist_sq_to(currentVertex);
                if(distanceFromBone < closestBoneDistance)
                {
                    closestBoneDistance = distanceFromBone;
                    closestBoneId = boneId;
                }
            }

            if(closestBoneId != -1)
                bones.push_back(closestBoneId);
        }
    }

    // Remove items from loaderSkeleton that have no samples, reparenting children.
    void removeEmptySurfaces(std::map<Bone::Id, BoneItem> &loaderSkeleton, const SampleSet::SampleSet &samples)
    {
        // Move the children of surfaces with no samples to its parent, so we don't have to
        // create empty surfaces.
        set<Bone::Id> emptySurfaces;
        for(auto &it: loaderSkeleton)
        {
            Bone::Id boneId = it.first;
            bool empty = samples._samples.find(boneId) == samples._samples.end() || samples._samples.at(boneId).nodes.empty();
            if(!empty)
                continue;
            emptySurfaces.insert(boneId);

            // Assign children of this surface to its parent.
            Bone::Id newParent = it.second.parent;
            for(auto &it2: loaderSkeleton)
            {
                if(it2.second.parent == boneId)
                    it2.second.parent = newParent;
            }
        }

        // Erase the empty nodes.
        for(Bone::Id boneId: emptySurfaces)
            loaderSkeleton.erase(boneId);
    }
}

void ImplicitCommand::init(MString skinClusterName)
{
    MStatus status = MStatus::kSuccess;

    MPlug skinClusterPlug = getOnePlugByName(skinClusterName);

    // Create entries in abs_skeleton for each influence object.  Each joint is placed at the same world
    // space position as the corresponding influence object.
    Abs_skeleton abs_skeleton;
    abs_skeleton.load(skinClusterPlug.node());

    // Create bones from the Abs_skeleton.  These are temporary and used only for sampling.
    // New, final bones will be created within the ImplicitSurfaces once we decide which ones
    // to create.
    std::map<Bone::Id, BoneItem> loaderSkeleton;
    loadBones(abs_skeleton, loaderSkeleton);

    // Load the skeleton.
    vector<shared_ptr<const Bone> > const_bones;
    for(auto &it: loaderSkeleton)
        const_bones.push_back(it.second.bone);
    shared_ptr<Skeleton> skeleton(new Skeleton(const_bones, abs_skeleton._parents));

    // Get the output of the skin cluster, which is what we'll sample.  We'll create
    // implicit surfaces based on the skin cluster in its current pose.
    std::unique_ptr<Mesh> mesh(createMeshFromSkinClusterOutput(skinClusterPlug.node()));

    // Create a list of the bones that each vertex should be included in.
    std::vector< std::vector<Bone::Id> > bonesPerVertex;
    clusterVerticesToBones(skinClusterPlug.node(), mesh.get(), loaderSkeleton, bonesPerVertex);

    VertToBoneInfo vertToBoneInfo(skeleton.get(), mesh.get(), bonesPerVertex);
    
    SampleSet::SampleSetSettings sampleSettings;

    // Get the default junction radius. XXX: this should be a parameter
    vertToBoneInfo.get_default_junction_radius(skeleton.get(), mesh.get(), sampleSettings.junction_radius);

    // Run the sampling for each joint.  The joints are in world space, so the samples will also be in
    // world space.
    SampleSet::SampleSet samples;
    for(Bone::Id bone_id: skeleton->get_bone_ids())
        samples.choose_hrbf_samples(mesh.get(), skeleton.get(), vertToBoneInfo, sampleSettings, bone_id);

    // Remove surfaces that didn't find any samples.
    removeEmptySurfaces(loaderSkeleton, samples);

    std::map<Bone::Id,float> hrbf_radius;
    vertToBoneInfo.get_default_hrbf_radius(skeleton.get(), mesh.get(), hrbf_radius);

    // Create a group to store all of the nodes we'll create.
    MObject mainGroup = DagHelpers::createTransform(skinClusterName + "Implicit", status); merr("createTransform");

    status = DagHelpers::lockTransforms(mainGroup); merr("lockTransforms");

    // Create an ImplicitBlend to combine the surfaces that we're creating together.
    MObject blendTransformNode;
    ImplicitBlend *blend = createShape<ImplicitBlend>(skinClusterName + "ImplicitBlend", &blendTransformNode);

    status = DagHelpers::lockTransforms(blendTransformNode); merr("lockTransforms");
    status = DagHelpers::setParent(mainGroup, blendTransformNode); merr("setParent");

    // The surfaces that we're creating, and their corresponding parents.
    vector<ImplicitSurface *> surfaces;
    vector<int> parent_index;
    map<Bone::Id, int> sourceBoneIdToIdx;
    MFn::kSkinClusterFilter;

    // Create an ImplicitSurface for each bone that has samples.
    for(auto &it: loaderSkeleton)
    {
        Bone::Id bone_id = it.first;
        BoneItem &bone_item = it.second;
        shared_ptr<Bone> bone = bone_item.bone;

        // The path to the influence object this surface was created for:
        const MDagPath &influenceObjectPath = bone_item.dagPath;

        // Figure out a name for the surface.
        MString surfaceName = influenceObjectPath.partialPathName();
        MObject transformNode;
        ImplicitSurface *surface = createShape<ImplicitSurface>(surfaceName + "Implicit", &transformNode);

        // Remember the offset in surfaces[] for this source bone ID.
        sourceBoneIdToIdx[bone->get_bone_id()] = (int) surfaces.size();

        surfaces.push_back(surface);

        status = DagHelpers::setParent(mainGroup, transformNode); merr("setParent");

        // This surface was created for a joint.  Get the transform for the joint associated
        // with this surface, and move the transform to the same place.  This positions the
        // implicit surface in the same place as the geometry it was sampled from.  This also
        // means that the samples are in the correct place: the samples are in the source mesh's
        // object space, and we're giving the surface that same world space.
        {
            MMatrix jointTransformMatrix = influenceObjectPath.exclusiveMatrix(&status); merr("influenceObjectPath.exclusiveMatrix");

            MFnTransform transform(transformNode, &status); merr("transform(transformNode)");

            status = transform.set(MTransformationMatrix(jointTransformMatrix)); merr("transform.set");

            // Parent this implicit surface's transform under the parent influence.
            const MDagPath &parentInfluenceObjectPath = bone_item.parentInfluence;
            assert(!parentInfluenceObjectPath.node().isNull());

            MFnDagNode dagNode(parentInfluenceObjectPath.node(), &status); merr("dagNode(parentInfluenceObjectPath)");
            MString pathName = transform.fullPathName(&status); merr("transform.fullPathName");
            MString parentPathName = parentInfluenceObjectPath.fullPathName(&status); merr("parentInfluenceObjectPath.fullPathName");
            MString cmd("parentConstraint -mo \"" + parentPathName + "\" \"" + pathName + "\";");

            status = MGlobal::executeCommand(cmd); merr("parentConstraint");
        }

        // Store this surface's HRBF radius.
        MPlug hrbfRadiusPlug(surface->thisMObject(), ImplicitSurface::hrbfRadiusAttr);
        status = hrbfRadiusPlug.setFloat(hrbf_radius.at(bone_id)); merr("hrbfRadiusPlug.setFloat");

        // Grab the samples for this surface.
        SampleSet::InputSample inputSample;
        if(samples._samples.find(bone_id) != samples._samples.end())
            inputSample = samples._samples.at(bone_id);

        // We created the samples in world space.  Transform them into the object space of the transform.
        MMatrix jointTransformInvMatrix = influenceObjectPath.exclusiveMatrixInverse(&status); merr("influenceObjectPath.exclusiveMatrixInverse");

        Transfo worldToObjectMatrix = DagHelpers::MMatrixToTransfo(jointTransformInvMatrix);
        inputSample.transform(worldToObjectMatrix);

        // We set up a temporary bone for sampling, which loaded the bone direction, but we then
        // created a new bone in the surface.  That bone was never told its direction, so set it
        // now.  We don't need to set the origin, since surface bones always start at the origin.
        // It would be cleaner to avoid creating the extra bones in the first place, but we might
        // end up needing to delete bones instead (if they have no samples).
        status = surface->set_bone_direction(worldToObjectMatrix * bone_item.bone->_dir); merr("surface->set_bone_direction");

        // Store this surface's samples.
        status = surface->save_sampleset(inputSample); merr("save_sampleset");

        if(bone_item.parent != -1)
        {
            // The source bone has a parent.  If the parent is in the skeleton, we should have already
            // created it, so find the index of its output bone.
            const BoneItem &parentItem = loaderSkeleton.at(bone_item.parent);
            auto boneIt = sourceBoneIdToIdx.find(parentItem.bone->get_bone_id());
            if(boneIt != sourceBoneIdToIdx.end())
            {
                int parent_idx = (int) std::distance(sourceBoneIdToIdx.begin(), boneIt);
                parent_index.push_back(parent_idx);
            }
            else
            {
                parent_index.push_back(-1);
            }
        }
        else
            parent_index.push_back(-1);
    }

    int nextBlendInputIdx = 0;

    MDGModifier dgModifier;
    for(int i = 0; i < (int) surfaces.size(); ++i)
    {
        const ImplicitSurface *surface = surfaces[i];
        // Connect the ImplicitSurface's worldImplicit[0] output to the ImplicitBlend's ImplicitBlend::implicit inputs.
        MPlug worldSurfaceOutputArray(surface->thisMObject(), ImplicitSurface::worldImplicit);
        MPlug worldSurfaceOutput = worldSurfaceOutputArray.elementByLogicalIndex(0);
        MPlug blendSurfacesArray(blend->thisMObject(), ImplicitBlend::surfaces);
        MPlug blendSurface = blendSurfacesArray.elementByLogicalIndex(nextBlendInputIdx, &status); merr("blendSurfacesArray.elementByLogicalIndex");
        MPlug blendImplicitInput = blendSurface.child(ImplicitBlend::implicit, &status); merr("blendSurface.child");

        status = dgModifier.connect(worldSurfaceOutput, blendImplicitInput); merr("dgModifier.connect");

        // Tell the blend node about this bone's parent.  This is -1 if this is a root joint.
        MPlug parentJoint = blendSurface.child(ImplicitBlend::parentJoint, &status); merr("blendSurface.child");
        int parent_idx = parent_index[i];
        dgModifier.newPlugValueInt(parentJoint, parent_idx);
        ++nextBlendInputIdx;
    }

    status = dgModifier.doIt(); merr("dgModifier.doIt");

    // Return the path to the new blend node.  We don't need to return the names of the individual
    // surfaces, since they can be found by looking at the inputs into the blend node.
    MDagPath dagPath;
    status = MDagPath::getAPathTo(blend->thisMObject(), dagPath); merr("MDagPath::getAPathTo");
    MString path = dagPath.partialPathName(&status); merr("dagPath.partialPathName");

    appendToResult(path);
}

void ImplicitCommand::test(MString nodeName)
{
    ImplicitDeformer *deformer = getDeformerByName(nodeName);
}

MStatus ImplicitCommand::doIt(const MArgList &args)
{
    try {
        MStatus status;
        for(int i = 0; i < (int) args.length(); ++i)
        {
            if(args.asString(i, &status) == MString("-createShapes") && MS::kSuccess == status)
            {
                ++i;
                MString nodeName = args.asString(i, &status);
                if(status != MS::kSuccess) return status;

                init(nodeName);
            }
            else if(args.asString(i, &status) == MString("-updateBase") && MS::kSuccess == status)
            {
                ++i;
                MString nodeName = args.asString(i, &status);
                if(status != MS::kSuccess) return status;

                calculate_base_potential(nodeName);
            }
            else if(args.asString(i, &status) == MString("-test") && MS::kSuccess == status)
            {
                ++i;
                MString nodeName = args.asString(i, &status);
                if(status != MS::kSuccess) return status;

                test(nodeName);
            }
        }
        return MS::kSuccess;
    }
    catch (std::exception &e) {
        MGlobal::displayError(e.what());
        return MS::kFailure;
    }
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

    try {
        MFnPlugin plugin(obj, "", "1.0", "Any");

        status = plugin.registerData("ImplicitSurfaceData", ImplicitSurfaceData::id, ImplicitSurfaceData::creator);
        merr("registerData(ImplicitSurfaceData)");

        status = ImplicitSurfaceGeometryOverride::initialize();
        merr("ImplicitSurfaceGeometryOverride::initialize");

        status = plugin.registerShape("implicitSurface", ImplicitSurface::id, &ImplicitSurface::creator,
               &ImplicitSurface::initialize, &ImplicitSurfaceUI::creator, &ImplicitSurfaceGeometryOverride::drawDbClassification);
        merr("registerShape(implicitSurface)");

        status = plugin.registerShape("ImplicitBlend", ImplicitBlend::id, &ImplicitBlend::creator,
               &ImplicitBlend::initialize, &ImplicitSurfaceUI::creator, &ImplicitSurfaceGeometryOverride::drawDbClassification);
        merr("registerShape(ImplicitBlend)");

        status = plugin.registerNode("implicitDeformer", ImplicitDeformer::id, ImplicitDeformer::creator, ImplicitDeformer::initialize, MPxNode::kDeformerNode);
        merr("registerNode(implicitDeformer)");

        status = plugin.registerCommand("implicitSkin", ImplicitCommand::creator);
        merr("registerCommand(implicitSkin)");
    }
    catch (std::exception &e) {
        // We don't try to clean up registrations if we've partially registered, but let's at least
        // shut down CUDA.
        Cuda_ctrl::cleanup();
        MGlobal::displayError(e.what());
        return MS::kFailure;
    }

    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj)
{
    try {
        MStatus status;

        Cuda_ctrl::cleanup();

        MFnPlugin plugin(obj);

        status = plugin.deregisterNode(ImplicitDeformer::id);
        merr("deregisterNode(ImplicitDeformer)");

        status = plugin.deregisterNode(ImplicitBlend::id);
        merr("deregisterNode(ImplicitBlend)");

        status = plugin.deregisterNode(ImplicitSurface::id);
        merr("deregisterNode(ImplicitSurface)");

        status = ImplicitSurfaceGeometryOverride::uninitialize();
        merr("ImplicitSurfaceGeometryOverride::uninitialize");

        status = plugin.deregisterCommand("implicitSkin");
        merr("deregisterCommand(implicitSkin)");

        status = plugin.deregisterData(ImplicitSurfaceData::id);
        merr("deregisterData(ImplicitSurfaceData)");
    }
    catch (std::exception &e) {
        MGlobal::displayError(e.what());
        return MS::kFailure;
    }

    return MS::kSuccess;
}
