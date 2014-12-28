#include "maya_data.hpp"

#include <assert.h>

#include <maya/MItMeshVertex.h>
#include <maya/MItMeshPolygon.h>
#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MFnSkinCluster.h>
#include <maya/MMatrix.h>
#include "maya/maya_helpers.hpp"

using namespace std;

MStatus MayaData::load_mesh(MObject inputObject, Loader::Abs_mesh &mesh)
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

MStatus MayaData::loadJointTransformsFromSkinCluster(MPlug skinClusterPlug, vector<MMatrix> &out)
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
        bone._name = "";

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
