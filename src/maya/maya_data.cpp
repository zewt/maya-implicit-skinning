#define NO_CUDA

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

MStatus MayaData::loadSkeletonHierarchyFromSkinCluster(MObject skinClusterNode, vector<int> &parentIndexes)
{
    MStatus status = MStatus::kSuccess;

    // Get the influence objects (joints) for the skin cluster.
    // Convert to a vector.
    vector<MDagPath> joints;
    status = DagHelpers::getMDagPathsFromSkinCluster(skinClusterNode, joints);
    if(status != MS::kSuccess) return status;

    // Create the bones.
    for(int i = 0; i < joints.size(); ++i)
    {
        // Find this bone's closest ancestor to be its parent.  If it has no ancestors, use the root.
        const MDagPath &dagPath = joints[i];
        int parentIdx = DagHelpers::findClosestAncestor(joints, dagPath);
        parentIndexes.push_back(parentIdx);
    }

    return MStatus::kSuccess;
}
