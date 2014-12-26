#include <string.h>
#include <maya/MIOStream.h>
#include <math.h>
#include <assert.h>

#include <maya/MPxDeformerNode.h> 
#include <maya/MItGeometry.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MItMeshVertex.h>
#include <maya/MItMeshEdge.h>
#include <maya/MPxLocatorNode.h> 
#include <maya/MDagPath.h>
#include <maya/MDagPathArray.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnMatrixData.h>

#include <maya/MFnPlugin.h>
#include <maya/MFnDependencyNode.h>
#include <maya/MFnSkinCluster.h>

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

class ImplicitSkinDeformer: public MPxDeformerNode
{
public:
    static MTypeId id;

    virtual ~ImplicitSkinDeformer() { }

    static void *creator() { return new ImplicitSkinDeformer(); }
    static MStatus initialize();

    MStatus accessoryNodeSetup(MDagModifier& cmd);

//    MStatus deform(MDataBlock &block, MItGeometry &iter, const MMatrix &mat, unsigned int multiIndex);
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock);

private:
    PluginInterface pluginInterface;

    static MObject  offsetMatrix;     // offset center and axis
};

MTypeId ImplicitSkinDeformer::id( 0x11229090 );
MObject ImplicitSkinDeformer::offsetMatrix;

MStatus ImplicitSkinDeformer::initialize()
{
    MFnMatrixAttribute mAttr;
    offsetMatrix=mAttr.create("locateMatrix", "lm");
    mAttr.setStorable(false);
    mAttr.setConnectable(true);

    //  deformation attributes
    addAttribute(offsetMatrix);

    attributeAffects(ImplicitSkinDeformer::offsetMatrix, ImplicitSkinDeformer::outputGeom);

    return MStatus::kSuccess;
}

MStatus getConnectedPlugWithName(MPlug inputPlug, string name, MPlug &result)
{
    MStatus status = MStatus::kSuccess;

    MFnDependencyNode inputPlugDep(inputPlug.node());
    MObject geomObj = inputPlugDep.attribute(name.c_str(), &status);
    if(status != MS::kSuccess)
    {
        fprintf(stderr, "error finding inputGeom\n");
        return status;
    }

    MPlug geomObjPlug(inputPlug.node(), geomObj);
    fprintf(stderr, "inputGeom: %s\n", geomObjPlug.name().asChar());

    
    MPlugArray connPlugs;
    geomObjPlug.connectedTo(connPlugs, true, false);
    int connLength = connPlugs.length();
    if(connLength == 0) {
        fprintf(stderr, "no connection\n");
        return status;
    }

    result = connPlugs[0];
    return MStatus::kSuccess;
}

static int compare_length(const MDagPath &lhs, const MDagPath &rhs) { return lhs.fullPathName().length() < rhs.fullPathName().length(); }

// Find the nearest ancestor to path.  "a|b|c" is an ancestor of "a|b|c|d|e|f".
// If no nodes are an ancestor of path, return -1.
int find_closest_ancestor(const vector<MDagPath> &dagPaths, MDagPath dagPath)
{
    string path = dagPath.fullPathName().asChar();

    int best_match = -1;
    int best_match_length = -1;
    for(size_t i = 0; i < dagPaths.size(); ++i) {
        string parentPath = dagPaths[i].fullPathName().asChar();
        if(parentPath == path)
            continue;

        // If path doesn't begin with this path plus |, it's not an ancestor.
        string compareTo = parentPath + "|";
        if(path.compare(0, compareTo.size(), compareTo, 0, compareTo.size()) != 0)
            continue;

        if((int) parentPath.size() > best_match_length)
        {
            best_match = (int) i;
            best_match_length = (int) parentPath.size();
        }
    }
    return best_match;
}

Loader::CpuTransfo MMatrixToCpuTransfo(const MMatrix &dagMat)
{
    Loader::CpuTransfo mat;
    mat[0] = (float) dagMat[0][0];
    mat[1] = (float) dagMat[1][0];
    mat[2] = (float) dagMat[2][0];
    mat[3] = (float) dagMat[3][0];
    mat[4] = (float) dagMat[0][1];
    mat[5] = (float) dagMat[1][1];
    mat[6] = (float) dagMat[2][1];
    mat[7] = (float) dagMat[3][1];
    mat[8] = (float) dagMat[0][2];
    mat[9] = (float) dagMat[1][2];
    mat[10] = (float) dagMat[2][2];
    mat[11] = (float) dagMat[3][2];
    mat[12] = (float) dagMat[0][3];
    mat[13] = (float) dagMat[1][3];
    mat[14] = (float) dagMat[2][3];
    mat[15] = (float) dagMat[3][3];
    return mat;
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
        MPoint point = meshIt.position(MSpace::kWorld, &status);
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
        status = polyIt.getTriangles(trianglePoints, triangleIndexes, MSpace::kWorld)		;
        if(status != MS::kSuccess) return status;

        assert(triangleIndexes.length() % 3 == 0);
        for(size_t triIdx = 0; triIdx < triangleIndexes.length(); triIdx += 3)
        {
            Loader::Tri_face f;
            for(unsigned faceIdx = 0; faceIdx < 3; ++faceIdx)
            {
                f.v[faceIdx] = triangleIndexes[faceIdx];
                f.n[faceIdx] = triangleIndexes[faceIdx]; // XXX?
            }

            mesh._triangles.push_back(f);
        }
    }

    return MS::kSuccess;
}

MStatus loadSkeletonFromSkinCluster(const MFnSkinCluster &skinCluster, Loader::Abs_skeleton &skeleton)
{
    // Get the influence objects (joints) for the skin cluster.
    MStatus status = MStatus::kSuccess;
    MDagPathArray paths;
    skinCluster.influenceObjects(paths, &status);
    if(status != MS::kSuccess)
        return status;

    // Convert to a vector.
    vector<MDagPath> joints;
    for(unsigned i = 0; i < paths.length(); ++i) 
        joints.push_back(paths[i]);

    // Sort the influence objects by the length of their full path.  Since the name of
    // an object is prefixed by its parents, eg. "parent1|parent2|object", this guarantees
    // that a parent is before all of its children.
    sort(joints.begin(), joints.end(), compare_length);

    // Create a root bone.
    // XXX put this back
    skeleton._root = 0;
/*    {
        Loader::Abs_bone bone;
        bone._name = "root";
        bone._frame = Loader::CpuTransfo::identity();
        skeleton._bones.push_back(bone);
        skeleton._sons.push_back(std::vector<int>());
        skeleton._parents.push_back(-1);
        skeleton._root = 0;
    }
    */
    // Create the bones.
    for(int i = 0; i < joints.size(); ++i)
    {
        const MDagPath &dagPath = joints[i];
        int boneIdx = (int) skeleton._bones.size(); 
        string jointName = dagPath.fullPathName().asChar();

        // Fill in the bone.  We don't need to calculate _length, since compute_bone_lengths will
        // do it below.
        Loader::Abs_bone bone;
        bone._name = jointName;
        bone._frame = MMatrixToCpuTransfo(dagPath.inclusiveMatrix());
        skeleton._bones.push_back(bone);

        // Find this bone's closest ancestor to be its parent.  If it has no ancestors, use the root.
        int parentIdx = find_closest_ancestor(joints, dagPath);
        if(i == 0)
            parentIdx = -1; // XXX
/*        if(parentIdx == -1)
            parentIdx = 0;
        else
            parentIdx++; // skip the root bone added above
            */
        skeleton._parents.push_back(parentIdx);

        // Add this bone as a child of its parent.
        skeleton._sons.push_back(std::vector<int>());
        if(parentIdx != -1)
            skeleton._sons[parentIdx].push_back(boneIdx);
    }

    skeleton.compute_bone_lengths();
    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::compute(const MPlug &plug, MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;
        
    if (plug.attribute() != outputGeom)
        return MStatus::kUnknownParameter;

    // The logical index of the output that we've been told to compute:
    unsigned int logicalOutputIndex = plug.logicalIndex();

    // Get the corresponding input plug for this output:
    // MObject inputAttr = MFnDependencyNode(plug.node()).attribute("input", &status); // == this.input
    MPlug inputPlug(plug.node(), input);
    fprintf(stderr, "inPlug %s %i\n", inputPlug.name().asChar(), inputPlug.numConnectedElements());

    // Select input[index].
    inputPlug.selectAncestorLogicalIndex(logicalOutputIndex, input);

    // Select the connection to input[index], which should be a groupParts node.
    MPlug groupParts;
    status = getConnectedPlugWithName(inputPlug, "inputGeometry", groupParts);
    if(groupParts.node().apiType() != MFn::kGroupParts)
    {
        fprintf(stderr, "Expected to find a groupParts above the deformer, found a %s instead\n", groupParts.node().apiTypeStr());
        return MStatus::kUnknownParameter;
    }

    // The input geometry plugged into the groupParts node.  This is normally the next
    // deformer in the chain, or the initial geometry if we're the first deformer.  We
    // expect to be after skinning, since we need to work on geometry after skinning.
    // XXX: There might be other deformers between us and the skinCluster; skip past them
    // the deformers until we find it.  However, this all needs to be moved out of compute()
    // and into a separate method, and that should probably be done in Python and not native
    // anyway, so let's solve that when that happens.
    MPlug skinClusterPlug;
    status = getConnectedPlugWithName(groupParts, "inputGeometry", skinClusterPlug);
    if(skinClusterPlug.node().apiType() != MFn::kSkinClusterFilter)
    {
        fprintf(stderr, "Expected to find a skinCluster above the deformer, found a %s instead\n", skinClusterPlug.node().apiTypeStr());
        return MStatus::kUnknownParameter;
    }


    MFnSkinCluster skinCluster(skinClusterPlug.node(), &status);
    if(status != MS::kSuccess) return status;

    Loader::Abs_skeleton skeleton;
    status = loadSkeletonFromSkinCluster(skinCluster, skeleton);
    if(status != MS::kSuccess) return status;








    MArrayDataHandle inputArray = dataBlock.inputArrayValue(input, &status);
    if(status != MS::kSuccess) return status;
    inputArray.jumpToElement(logicalOutputIndex);

    MDataHandle inputGeomData = inputArray.inputValue(&status);
    if(status != MS::kSuccess) return status;

    MDataHandle inputGeomDataHandle = inputGeomData.child(inputGeom);














    MObject inputObject = inputGeomDataHandle.data();
    if (inputObject.apiType() != MFn::kMeshData) {
        // XXX: only meshes are supported
        return MS::kFailure;
    }

    MFnMesh myMesh(inputObject, &status);
    if(status != MS::kSuccess) {
        printf("Couldn't get input geometry as mesh\n");
        return status;
    }

    Loader::Abs_mesh mesh;
    status = load_mesh(inputObject, mesh);
    if(status != MS::kSuccess) return status;

    vector<Loader::Vec3> result_verts;
    pluginInterface.go(mesh, skeleton, result_verts);






    MDataHandle hInput = dataBlock.inputValue(inputPlug);
//    MDataHandle inputGeomDataHandle = hInput.child(inputGeom);

    // The groupId of this groupParts node:
    MDataHandle hGroup = hInput.child(groupId);
    unsigned int groupId = hGroup.asLong();
    MDataHandle hOutput = dataBlock.outputValue(plug);
    hOutput.copy(inputGeomDataHandle);

    // do the deformation
    MItGeometry iter(hOutput, groupId, false);
    int idx = 0;
    for ( ; !iter.isDone(); iter.next()) {
        if(idx >= result_verts.size())
            break;
        MPoint pt = iter.position();

        Loader::Vec3 v = result_verts[idx];
        pt = MPoint(v.x, v.y, v.z);
        iter.setPosition(pt, MSpace::kWorld);
        idx++;
    }

    return MStatus::kSuccess;
}

MStatus ImplicitSkinDeformer::accessoryNodeSetup(MDagModifier& cmd)
{
    return MS::kSuccess;
/*
    MStatus result;

    // hook up the accessory node
    MObject objLoc = cmd.createNode(MString("locator"), MObject::kNullObj, &result);
    if (result != MS::kSuccess)
        return result;

    MFnDependencyNode fnLoc(objLoc);
    MString attrName;
    attrName.set("matrix");
    MObject attrMat = fnLoc.attribute(attrName);

    return cmd.connect(objLoc, attrMat, thisMObject(), ImplicitSkinDeformer::offsetMatrix);
*/
}

MStatus initializePlugin(MObject obj)
{
    PluginInterface::init();

    MFnPlugin plugin(obj, "", "1.0", "Any");
    return plugin.registerNode("implicit", ImplicitSkinDeformer::id, ImplicitSkinDeformer::creator, ImplicitSkinDeformer::initialize, MPxNode::kDeformerNode);
}

MStatus uninitializePlugin(MObject obj)
{
    PluginInterface::shutdown();

    MFnPlugin plugin(obj);
    return plugin.deregisterNode(ImplicitSkinDeformer::id);
}
