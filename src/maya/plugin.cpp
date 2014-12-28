#include <string.h>
#include <maya/MIOStream.h>
#include <math.h>
#include <assert.h>

#include <maya/MPxDeformerNode.h> 
#include <maya/MPxCommand.h>
#include <maya/MPxData.h>
#include <maya/MItGeometry.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshEdge.h>
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixData.h>

#include <maya/MFnDagNode.h>
#include <maya/MFnPlugin.h>
#include <maya/MFnTransform.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnSkinCluster.h>
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
#include "loader_mesh.hpp"
#include "loader_skel.hpp"

#include "Interface.hpp"

#include <algorithm>
#include <map>
using namespace std;

// Make sure that this is a string which isn't a valid DAG name.
#define ROOT_BONE_NAME ""

#if 0
class ImplicitSkinDeformerData: public MPxData
{
public:
    static MTypeId id;
    MTypeId typeId() const { return id; }
    MString name() const { return "ImplicitSkinDeformerData"; }
    void copy(const MPxData &src)
    {
        const ImplicitSkinDeformerData &srcData = dynamic_cast<const ImplicitSkinDeformerData &>(src);
//        pluginInterface = srcData.pluginInterface;
    }

    static void *creator() { return new ImplicitSkinDeformerData(); }


};
MTypeId ImplicitSkinDeformerData::id( 0x11229091 ); // XXX
#endif

class ImplicitSkinDeformer: public MPxDeformerNode
{
public:
    static MTypeId id;

    PluginInterface pluginInterface;

    virtual ~ImplicitSkinDeformer() { }

    static void *creator() { return new ImplicitSkinDeformer(); }
    static MStatus initialize();

//    MStatus deform(MDataBlock &block, MItGeometry &iter, const MMatrix &mat, unsigned int multiIndex);
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock);

    static MObject dataAttr;
    static MObject worldMatrixInverse;
/*
    static MStatus getOrCreateDeformerDataFromObj(MObject implicitDeformer, ImplicitSkinDeformerData *&data)
    {
        MStatus status;

        MPlug dataPlug(implicitDeformer, dataAttr);

        MObject dataObj;
        dataPlug.getValue(dataObj);
        if(dataObj.isNull()) {
            // No object exists yet, so create one.
            MFnPluginData pluginData;
            dataObj = pluginData.create(ImplicitSkinDeformerData::id);

            status = dataPlug.setValue(dataObj);
            if(status != MS::kSuccess) return status;
        }

        MFnPluginData pluginData(dataObj, &status);
        if(status != MS::kSuccess) return status;

        MTypeId typeId = pluginData.typeId(&status);
        if(status != MS::kSuccess) return status;
        if(typeId != ImplicitSkinDeformerData::id)
            return MStatus::kFailure;

        data = (ImplicitSkinDeformerData *) pluginData.data();
        return MStatus::kSuccess;
    }
    */
};

MTypeId ImplicitSkinDeformer::id( 0x11229090 ); // XXX
MObject ImplicitSkinDeformer::dataAttr;
MObject ImplicitSkinDeformer::worldMatrixInverse;

MStatus ImplicitSkinDeformer::initialize()
{
    MStatus status = MStatus::kSuccess;

/*    MFnTypedAttribute typedAttr;
    dataAttr = typedAttr.create("data", "d", ImplicitSkinDeformerData::id, MObject::kNullObj, &status);
    addAttribute(dataAttr);
    attributeAffects(ImplicitSkinDeformer::dataAttr, ImplicitSkinDeformer::outputGeom);*/

    MFnMatrixAttribute mAttr;
    worldMatrixInverse = mAttr.create("worldMatrixInverse", "lm");

    addAttribute(worldMatrixInverse);

    attributeAffects(ImplicitSkinDeformer::worldMatrixInverse, ImplicitSkinDeformer::outputGeom);

    return MStatus::kSuccess;
}



MStatus load_mesh(MObject inputObject, Loader::Abs_mesh &mesh)
{
    MStatus status = MS::kSuccess;
    MItMeshVertex meshIt(inputObject, &status);
    if(status != MS::kSuccess) return status;

    // Load vertices and normals.
    int num_verts = meshIt.count(&status);
    if(status != MS::kSuccess) return status;

    mesh._vertices.resize(num_verts);
    mesh._normals.resize(num_verts);

    int idx = 0;
    for ( ; !meshIt.isDone(); meshIt.next())
    {
        MPoint point = meshIt.position(MSpace::kObject, &status);
        if(status != MS::kSuccess) return status;

        mesh._vertices[idx] = Loader::Vertex((float)point.x, (float)point.y, (float)point.z);

        // XXX: What are we supposed to do with unshared normals?
        MVectorArray normalArray;
        status = meshIt.getNormals(normalArray, MSpace::kObject);
        if(status != MS::kSuccess) return status;

        MVector normal = normalArray[0];
        mesh._normals[idx] = Loader::Normal((float)normal[0], (float)normal[1], (float)normal[2]);

        ++idx;
    }

    // Load tris using Maya's triangulation.
    MItMeshPolygon polyIt(inputObject);
    int num_faces = polyIt.count(&status);
    if(status != MS::kSuccess) return status;

    for ( ; !polyIt.isDone(); polyIt.next())
    {
        if(!polyIt.hasValidTriangulation())
        {
            int idx = polyIt.index(&status);
            printf("Warning: Polygon with index %i doesn't have a valid triangulation", idx);
            continue;
        }

        MPointArray trianglePoints;
        MIntArray triangleIndexes;
        status = polyIt.getTriangles(trianglePoints, triangleIndexes, MSpace::kObject);
        if(status != MS::kSuccess) return status;

        assert(triangleIndexes.length() % 3 == 0);
        for(int triIdx = 0; triIdx < (int) triangleIndexes.length(); triIdx += 3)
        {
            Loader::Tri_face f;
            for(int faceIdx = 0; faceIdx < 3; ++faceIdx)
            {
                f.v[faceIdx] = triangleIndexes[triIdx+faceIdx];
                f.n[faceIdx] = triangleIndexes[triIdx+faceIdx];
            }

            mesh._triangles.push_back(f);
        }
    }

    return MS::kSuccess;
}


/*
 * Given a skinCluster plug, create an Abs_skeleton.
 *
 * If getBindPositions is false, use the current positions of the joints, eg. the worldMatrix of the influence
 * objects.  If it's true, return the positions of the joints at bind time, which is the inverse of
 * bindPreMatrix on the skinCluster.
 *
 * If worldToObjectSpaceMat is identity, the joints are returned in world space.  worldToObjectSpaceMat can
 * be used to return joints in a different coordinate system.
 */
MStatus loadSkeletonFromSkinCluster(MPlug skinClusterPlug, Loader::Abs_skeleton &skeleton, MMatrix worldToObjectSpaceMat, bool getBindPositions)
{
    MStatus status = MStatus::kSuccess;

    // Get the influence objects (joints) for the skin cluster.
    // Convert to a vector.
    vector<MDagPath> joints;
    status = DagHelpers::getMDagPathsFromSkinCluster(skinClusterPlug, joints);
    if(status != MS::kSuccess) return status;

    // Create a root bone.
    {
        // The root bone na
        Loader::Abs_bone bone;
        bone._name = ROOT_BONE_NAME;

        // Create this joint at the origin in world space, and transform it to object space like the
        // ones we create below.  (Multiplying by identity doesn't do anything; it's only written this
        // way for clarity.)
        MMatrix jointObjectMat = MMatrix::identity * worldToObjectSpaceMat;
        bone._frame = DagHelpers::MMatrixToCpuTransfo(jointObjectMat);

        skeleton._bones.push_back(bone);
        skeleton._sons.push_back(std::vector<int>());
        skeleton._parents.push_back(-1);
        skeleton._root = 0;
    }

    // Create the bones.
    for(int i = 0; i < joints.size(); ++i)
    {
        const MDagPath &dagPath = joints[i];
        int boneIdx = (int) skeleton._bones.size(); 
        string jointName = dagPath.fullPathName().asChar();

        // If bind positions have been requested, get them from the bindPreMatrix.  This is the (inverse)
        // world transform of the joint at the time it was bound.  Otherwise, just get the current transform.
        MMatrix jointWorldMat;
        if(getBindPositions)
        {
            MFnDependencyNode skinClusterDep(skinClusterPlug.node());
            const MObject bindPreMatrixObject = skinClusterDep.attribute("bindPreMatrix", &status);
            if(status != MS::kSuccess) return status;
            
            MPlug bindPreMatrixArray(skinClusterPlug.node(), bindPreMatrixObject);
            MPlug bindPreMatrixElem = bindPreMatrixArray[i];

            MMatrix bindPreMatrix = DagHelpers::getMatrixFromPlug(bindPreMatrixElem, &status);

            // This is the inverse matrix.  We want the forwards matrix, so un-invert it.
            jointWorldMat = bindPreMatrix.inverse();
        }
        else
        {
            jointWorldMat = dagPath.inclusiveMatrix();
        }

        MMatrix jointObjectMat = jointWorldMat * worldToObjectSpaceMat;

        // Fill in the bone.  We don't need to calculate _length, since compute_bone_lengths will
        // do it below.
        Loader::Abs_bone bone;
        bone._name = jointName;
        bone._frame = DagHelpers::MMatrixToCpuTransfo(jointObjectMat);
        skeleton._bones.push_back(bone);

        // Find this bone's closest ancestor to be its parent.  If it has no ancestors, use the root.
        int parentIdx = DagHelpers::findClosestAncestor(joints, dagPath);
        if(parentIdx == -1)
            parentIdx = 0;
        else
            parentIdx++; // skip the root bone added above

        skeleton._parents.push_back(parentIdx);

        // Add this bone as a child of its parent.
        skeleton._sons.push_back(std::vector<int>());
        if(parentIdx != -1)
            skeleton._sons[parentIdx].push_back(boneIdx);
    }

    skeleton.compute_bone_lengths();
    return MStatus::kSuccess;
}

MStatus loadJointTransformsFromSkinCluster(MPlug skinClusterPlug, vector<MMatrix> &out)
{
    MStatus status = MStatus::kSuccess;

    MFnSkinCluster skinCluster(skinClusterPlug.node(), &status);
    if(status != MS::kSuccess) return status;

    // Get the influence objects (joints) for the skin cluster.
    vector<MDagPath> joints;
    status = DagHelpers::getMDagPathsFromSkinCluster(skinClusterPlug, joints);
    if(status != MS::kSuccess) return status;

    // Create a dummy first bone, to correspond with the root bone.
    out.push_back(MMatrix::identity);
    for(int i = 0; i < joints.size(); ++i)
        out.push_back(joints[i].inclusiveMatrix());

    return MStatus::kSuccess;
}

MStatus findAncestorDeformer(MPlug inputPlug, MFn::Type type, MPlug &resultPlug)
{
    while(true)
    {
        MStatus status = DagHelpers::getConnectedPlugWithName(inputPlug, "inputGeometry", inputPlug);
        if(status != MS::kSuccess)
            return status;

        if(inputPlug.node().apiType() == type)
        {
            resultPlug = inputPlug;
            return MStatus::kSuccess;
        }
    }
}

MStatus getInputGeometryForSkinClusterPlug(MPlug skinClusterPlug, MObject &plug)
{
    // This gets the original geometry, not the geometry input into the skinCluster:
    /*
    MObjectArray inputGeometries;
    status = skinCluster.getInputGeometry(inputGeometries);
    if(status != MS::kSuccess)
        return status;

    if(inputGeometries.length() == 0)
    {
        printf("skinCluster has no input geometry\n");
        return MS::kFailure;
    }

    // skinClusters don't support more than one input geometry.
    MObject inputValue = inputGeometries[0];
    */

    MStatus status = MStatus::kSuccess;
    MObject inputAttr = MFnDependencyNode(skinClusterPlug.node()).attribute("input", &status);
    if(status != MS::kSuccess) return status;

    MPlug inputPlug(skinClusterPlug.node(), inputAttr);
    if(status != MS::kSuccess) return status;

    // Select input[0].  skinClusters only support a single input.
    inputPlug = inputPlug.elementByPhysicalIndex(0, &status);
    if(status != MS::kSuccess) return status;

    MFnDependencyNode inputPlugDep(inputPlug.node());
    MObject inputObject = inputPlugDep.attribute("inputGeometry", &status);
    if(status != MS::kSuccess) return status;
            
    inputPlug = inputPlug.child(inputObject, &status);
    if(status != MS::kSuccess) return status;

    status = inputPlug.getValue(plug);
    if(status != MS::kSuccess) return status;

    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;
        
    if (plug.attribute() != outputGeom)
        return MStatus::kUnknownParameter;

    // If implicit -setup hasn't been run yet, stop.  XXX: saving/loading
    if(!pluginInterface.is_setup())
        return MStatus::kSuccess;

    // The logical index of the output that we've been told to compute:
    unsigned int logicalOutputIndex = plug.logicalIndex();

    // We only support a single input, like skinCluster.
    if(logicalOutputIndex > 0)
        MStatus status = MStatus::kSuccess;

    // Get the corresponding input plug for this output:
    // MObject inputAttr = MFnDependencyNode(plug.node()).attribute("input", &status); // == this.input
    MPlug inputPlug(plug.node(), input);

    // Select input[index].
    inputPlug.selectAncestorLogicalIndex(logicalOutputIndex, input);

    // Find the skinCluster deformer node above us.
    MPlug skinClusterPlug;
    status = findAncestorDeformer(inputPlug, MFn::kSkinClusterFilter, skinClusterPlug);
    if(status != MS::kSuccess)
    {
        printf("Couldn't find a skinCluster deformer.  Is the node skinned?\n");
        return status;
    }


    // XXX: We need to declare our dependency on the skinCluster, probably via skinCluster.baseDirty.
    // XXX: Also, skinCluster.preBindMatrix, but that one's an array




    // Get the worldMatrixInverse attribute.
    MPlug worldMatrixInversePlug(thisMObject(), worldMatrixInverse);
    MMatrix worldToObjectSpaceMat = DagHelpers::getMatrixFromPlug(worldMatrixInversePlug, &status);



    // Get the joint world matrix transforms.
    vector<MMatrix> jointTransformsWorldSpace;
    status = loadJointTransformsFromSkinCluster(skinClusterPlug, jointTransformsWorldSpace);
    if(status != MS::kSuccess) return status;

    // XXX: Copy this data into an attribute in this node so we don't pull it externally.
    MFnDependencyNode skinClusterDep(skinClusterPlug.node());
    const MObject bindPreMatrixObject = skinClusterDep.attribute("bindPreMatrix", &status);
    if(status != MS::kSuccess) return status;
            
    MPlug bindPreMatrixArray(skinClusterPlug.node(), bindPreMatrixObject);

    // Update the skeleton.  XXX: fix dependency handling

    vector<Loader::CpuTransfo> bone_positions;

    // The root joint is a dummy, and doesn't correspond with a Maya transform.
    bone_positions.push_back(Loader::CpuTransfo::identity());

    for(int i = 1; i < jointTransformsWorldSpace.size(); ++i)
    {
        // We need to get the change to the joint's transformation compared to when it was bound.
        // Maya gets this by multiplying the current worldMatrix against the joint's bindPreMatrix.
        // However, it's doing that in world space; we need it in object space.
        // XXX: There's probably a much simpler way to do this.
        MMatrix bindPreMatrixWorldSpace = DagHelpers::getMatrixFromPlug(bindPreMatrixArray[i-1], &status); // original inverted world space transform
        MMatrix bindMatrixWorldSpace = bindPreMatrixWorldSpace.inverse();                // original (non-inverted) world space transform
        MMatrix bindMatrixObjectSpace = bindMatrixWorldSpace * worldToObjectSpaceMat;    // original object space transform
        MMatrix bindMatrixObjectSpaceInv = bindMatrixObjectSpace.inverse();              // original inverted object space transform
        MMatrix currentTransformObjectSpace = jointTransformsWorldSpace[i] * worldToObjectSpaceMat; // current object space transform
        MMatrix changeToTransform = bindMatrixObjectSpaceInv * currentTransformObjectSpace; // joint transform relative to bind pose in object space
        
        bone_positions.push_back(DagHelpers::MMatrixToCpuTransfo(changeToTransform));
    }

    pluginInterface.update_skeleton(bone_positions);




    MArrayDataHandle inputArray = dataBlock.inputArrayValue(input, &status);
    if(status != MS::kSuccess) return status;
    inputArray.jumpToElement(logicalOutputIndex);

    MDataHandle inputGeomData = inputArray.inputValue(&status);
    if(status != MS::kSuccess) return status;

    MDataHandle inputGeomDataHandle = inputGeomData.child(inputGeom);







    // Update the vertex data.  We read all geometry, not just the set (if any) that we're being
    // applied to, so the algorithm can see the whole mesh.
    MDataHandle hInput = dataBlock.inputValue(inputPlug);

    // The groupId of this groupParts node:
    MDataHandle hGroup = hInput.child(groupId);
    unsigned int groupId = hGroup.asLong();
    MDataHandle hOutput = dataBlock.outputValue(plug);
    hOutput.copy(inputGeomDataHandle);



    {
        MItGeometry geomIter(hOutput, true);
        MPointArray points;
        geomIter.allPositions(points, MSpace::kObject);


        vector<Loader::Vertex> input_verts;
        input_verts.reserve(points.length());
        for(int i = 0; i < (int) points.length(); ++i)
            input_verts.push_back(Loader::Vertex((float) points[i].x, (float) points[i].y, (float) points[i].z));
        pluginInterface.update_vertices(input_verts);
    }

    // Run the algorithm.  XXX: If we're being applied to a set, can we reduce the work we're doing to
    // just those vertices?
    vector<Loader::Vec3> result_verts;
    pluginInterface.go(result_verts);

    // Copy out the vertices that we were actually asked to process.
    MItGeometry geomIter(hOutput, groupId, false);
    for ( ; !geomIter.isDone(); geomIter.next()) {
        int vertex_index = geomIter.index();

        Loader::Vec3 v = result_verts[vertex_index];
        MPoint pt = MPoint(v.x, v.y, v.z);
        geomIter.setPosition(pt, MSpace::kObject);
    }

    return MStatus::kSuccess;
}


MStatus setMatrixPlug(MPlug plug, MObject attr, MMatrix matrix)
{
    MStatus status;

    MFnDependencyNode plugDep(plug.node(), &status);
    if(status != MS::kSuccess) return status;

    // Find the worldMatrixInverse plug.
    MPlug attrPlug = plugDep.findPlug(attr, &status);
    if(status != MS::kSuccess) return status;

    // Create a MFnMatrixData holding the matrix.
    MFnMatrixData fnMat;
    MObject matObj = fnMat.create(&status);
    if(status != MS::kSuccess) return status;

    // Set the worldMatrixInverse attribute.
    fnMat.set(matrix);
    status = attrPlug.setValue(matObj);
    if(status != MS::kSuccess) return status;

    return MStatus::kSuccess;
}


class ImplicitCommand : public MPxCommand
{
public:
    virtual ~ImplicitCommand() { }
    MStatus doIt( const MArgList& );
//    MStatus redoIt();
//    MStatus undoIt();

    MStatus setup(MString nodeName);

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

MStatus ImplicitCommand::setup(MString nodeName)
{
    MStatus status;

    // Get the MPlug for the selected node.
    MPlug implicitPlug;
    status = getOnePlugByName(nodeName, implicitPlug);
    if(status != MS::kSuccess) return status;

    MFnDependencyNode implicitPlugDep(implicitPlug.node(), &status);
    if(status != MS::kSuccess) return status;

    // Verify that this is one of our nodes.
    {
        MTypeId type = implicitPlugDep.typeId(&status);
        if(status != MS::kSuccess) return status;

        if(type != ImplicitSkinDeformer::id)
        {
            displayError("Node not an implicitDeformer: " + nodeName);
            return MS::kFailure;
        }
    }

    ImplicitSkinDeformer *deformer = (ImplicitSkinDeformer *) implicitPlugDep.userNode(&status);
    if(status != MS::kSuccess) return status;

    // Create an MFnGeometryFilter on the ImplicitSkinDeformer.
    MFnGeometryFilter implicitGeometryFilter(implicitPlug.node(), &status);
    if(status != MS::kSuccess) return status;

    // Ask the MFnGeometryFilter for the MDagPath of the output, and pop to get to the transform
    // node.
    MDagPath implicitGeometryOutputDagPath;
    implicitGeometryFilter.getPathAtIndex(0, implicitGeometryOutputDagPath);
    implicitGeometryOutputDagPath.pop();

    // Get the inverse transformation, which is the transformation to go from world space to
    // object space.
    MFnTransform transformNode(implicitGeometryOutputDagPath.node());
    MMatrix worldToObjectSpaceMat = transformNode.transformationMatrix(&status).inverse();

    // Store worldToObjectSpaceMat on worldMatrixInverse.
    setMatrixPlug(implicitPlug, ImplicitSkinDeformer::worldMatrixInverse, worldToObjectSpaceMat);

    // XXX: We need to declare our dependency on the skinCluster, probably via skinCluster.baseDirty.
    // Find the skinCluster deformer node above the deformer.
    MPlug skinClusterPlug;
    status = findAncestorDeformer(implicitPlug, MFn::kSkinClusterFilter, skinClusterPlug);
    if(status != MS::kSuccess)
    {
        printf("Couldn't find a skinCluster deformer.  Is the node skinned?\n");
        return status;
    }

    MFnSkinCluster skinCluster(skinClusterPlug.node(), &status);
    if(status != MS::kSuccess) return status;

    // Load the skeleton.  Set getBindPositions to true so we get the positions the bones
    // had at bind time.  Note that we're still assuming that the current position of the
    // object is the same as bind time.  XXX: Is there anything we can do about that?
    // Maybe we should just use the current position of the geometry/joints at setup time.
    Loader::Abs_skeleton skeleton;
    status = loadSkeletonFromSkinCluster(skinClusterPlug, skeleton, worldToObjectSpaceMat, true);
    if(status != MS::kSuccess) return status;

    // Get the inputGeometry going into the skinCluster.  This is the mesh before skinning, which
    // we'll use to do initial calculations.  The bind-time positions of the joints we got above
    // should correspond with the pre-skinned geometry.
    MObject inputValue;
    status = getInputGeometryForSkinClusterPlug(skinClusterPlug, inputValue);
    if(status != MS::kSuccess) return status;
    fprintf(stderr, "inPlug %i\n", inputValue.apiType());


    if(!inputValue.hasFn(MFn::kMesh)) {
        // XXX: only meshes are supported
        return MS::kFailure;
    }

    // Load the input mesh.
    Loader::Abs_mesh mesh;
    status = load_mesh(inputValue, mesh);
    if(status != MS::kSuccess) return status;

    deformer->pluginInterface.setup(mesh, skeleton);

    return MS::kSuccess;
}

MStatus ImplicitCommand::doIt(const MArgList &args)
{
    MStatus status;
    for(int i = 0; i < (int) args.length(); ++i)
    {
        if(args.asString(i, &status) == MString("-setup") && MS::kSuccess == status)
        {
            ++i;
            MString nodeName = args.asString(i, &status);
            if(status != MS::kSuccess) return status;

            status = setup(nodeName);

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

    PluginInterface::init();

    MFnPlugin plugin(obj, "", "1.0", "Any");

//    status = plugin.registerData("ImplicitSkinDeformerData", ImplicitSkinDeformerData::id, ImplicitSkinDeformerData::creator);
//    if(status != MS::kSuccess) return status;

    status = plugin.registerNode("implicit", ImplicitSkinDeformer::id, ImplicitSkinDeformer::creator, ImplicitSkinDeformer::initialize, MPxNode::kDeformerNode);
    if(status != MS::kSuccess) return status;

    status = plugin.registerCommand( "implicit", ImplicitCommand::creator );
    if(status != MS::kSuccess) return status;

    return MS::kSuccess;
}

MStatus uninitializePlugin(MObject obj)
{
    MStatus status;

    PluginInterface::shutdown();

    MFnPlugin plugin(obj);

//    status = plugin.deregisterData(ImplicitSkinDeformerData::id);
//    if(status != MS::kSuccess) return status;

    status = plugin.deregisterNode(ImplicitSkinDeformer::id);
    if(status != MS::kSuccess) return status;

    status = plugin.deregisterCommand("implicit");
    if(status != MS::kSuccess) return status;

    return MS::kSuccess;
}
