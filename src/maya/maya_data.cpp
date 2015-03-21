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

void MayaData::load_mesh(MObject inputObject, Loader::Abs_mesh &mesh, MMatrix vertexTransform)
{
    MStatus status = MS::kSuccess;
    MItMeshVertex meshIt(inputObject, &status); merr("MItMeshVertex");

    // Load vertices and normals.
    int num_verts = meshIt.count(&status); merr("meshIt.count");

    mesh._vertices.resize(num_verts);
    mesh._normals.resize(num_verts);

    int idx = 0;
    for ( ; !meshIt.isDone(); meshIt.next())
    {
        MPoint point = meshIt.position(MSpace::kObject, &status); merr("meshIt.position");

        // If specified, transform the point from object space to another coordinate space.
        point = point * vertexTransform;

        mesh._vertices[idx] = Point_cu((float)point.x, (float)point.y, (float)point.z);

        // Load the vertex's normal.  If the normal has unshared normals, this will retrieve
        // the averaged normal.  Since we're normally blending soft surfaces, this is usually
        // okay.
        MVector normal;
        status = meshIt.getNormal(normal, MSpace::kObject); merr("meshIt.getNormal");
        normal = normal.transformAsNormal(vertexTransform);

        mesh._normals[idx] = Vec3_cu((float)normal[0], (float)normal[1], (float)normal[2]);

        ++idx;
    }

    // Load tris using Maya's triangulation.
    MItMeshPolygon polyIt(inputObject);
    int num_faces = polyIt.count(&status); merr("polyIt.count");

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
        status = polyIt.getTriangles(trianglePoints, triangleIndexes, MSpace::kObject); merr("polyIt.getTriangles");

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
}

void MayaData::loadSkeletonHierarchyFromSkinCluster(const std::map<int,MDagPath> &logicalIndexToInfluenceObjects, std::map<int,int> &logicalIndexToParentIdx)
{
    for(auto &it: logicalIndexToInfluenceObjects)
    {
        const MDagPath &dagPath = it.second;
        logicalIndexToParentIdx[it.first] = DagHelpers::findClosestAncestor(logicalIndexToInfluenceObjects, dagPath);
    }
}
