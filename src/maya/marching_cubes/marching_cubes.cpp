#define NO_CUDA

#include "marching_cubes.hpp"
#include "mesh_data.h"
#include "vertex_hash.h"
#include "tables.h"

#include <maya/MPointArray.h>
#include <maya/MIntArray.h>
#include <maya/MFnMesh.h>
#include <maya/MFnMeshData.h>

namespace MarchingCubes
{
    struct GridCell {
        MFloatPoint p[8];
        MColor c[8];
        float val[8];
    };

    MeshGeomVertex VertexLerp(const float isoLevel, const GridCell& cell, const int i1, const int i2) {
        // Linearly interpolate the position where an isosurface cuts
        // an edge between two vertices, each with their own scalar value
        MeshGeomVertex res;
        if(fabsf(isoLevel - cell.val[i1]) < 0.00001f) {
            res.pos = cell.p[i1];
            res.col = cell.c[i1];
            return res;
        }
        if(fabsf(isoLevel - cell.val[i2]) < 0.00001f) {
            res.pos = cell.p[i2];
            res.col = cell.c[i2];
            return res;
        }
    
        if(fabsf(cell.val[i1] - cell.val[i2]) < 0.00001f) {
            res.pos = cell.p[i1];
            res.col = cell.c[i1];
            return res;
        }

        float mu = (isoLevel - cell.val[i1]) / (cell.val[i2] - cell.val[i1]);

        res.pos = cell.p[i1] + mu * (cell.p[i2] - cell.p[i1]);
        res.col = cell.c[i1] + mu * (cell.c[i2] - cell.c[i1]);

        return res;
    }

    void polygonize(const GridCell &grid, VertexHash &vHash, MeshGeom &geom)
    {
        const float isoLevel = 0.5f;

        // Determine the index into the edge table which
        // tells us which vertices are inside of the surface
        int cubeIndex = 0;
        if(grid.val[0] < isoLevel) cubeIndex |= 1<<0;
        if(grid.val[1] < isoLevel) cubeIndex |= 1<<1;
        if(grid.val[2] < isoLevel) cubeIndex |= 1<<2;
        if(grid.val[3] < isoLevel) cubeIndex |= 1<<3;
        if(grid.val[4] < isoLevel) cubeIndex |= 1<<4;
        if(grid.val[5] < isoLevel) cubeIndex |= 1<<5;
        if(grid.val[6] < isoLevel) cubeIndex |= 1<<6;
        if(grid.val[7] < isoLevel) cubeIndex |= 1<<7;

        // Cube is entirely in/out of the surface
        if(edgeTable[cubeIndex] == 0)
            return;

        // Find the vertices where the surface intersects the cube
        MeshGeomVertex vertList[12];
        if(edgeTable[cubeIndex] & (1<<0))  vertList[0] = VertexLerp(isoLevel, grid, 0, 1);
        if(edgeTable[cubeIndex] & (1<<1))  vertList[1] = VertexLerp(isoLevel, grid, 1, 2);
        if(edgeTable[cubeIndex] & (1<<2))  vertList[2] = VertexLerp(isoLevel, grid, 2, 3);
        if(edgeTable[cubeIndex] & (1<<3))  vertList[3] = VertexLerp(isoLevel, grid, 3, 0);
        if(edgeTable[cubeIndex] & (1<<4))  vertList[4] = VertexLerp(isoLevel, grid, 4, 5);
        if(edgeTable[cubeIndex] & (1<<5))  vertList[5] = VertexLerp(isoLevel, grid, 5, 6);
        if(edgeTable[cubeIndex] & (1<<6))  vertList[6] = VertexLerp(isoLevel, grid, 6, 7);
        if(edgeTable[cubeIndex] & (1<<7))  vertList[7] = VertexLerp(isoLevel, grid, 7, 4);
        if(edgeTable[cubeIndex] & (1<<8))  vertList[8] = VertexLerp(isoLevel, grid, 0, 4);
        if(edgeTable[cubeIndex] & (1<<9))  vertList[9] = VertexLerp(isoLevel, grid, 1, 5);
        if(edgeTable[cubeIndex] & (1<<10)) vertList[10] = VertexLerp(isoLevel, grid, 2, 6);
        if(edgeTable[cubeIndex] & (1<<11)) vertList[11] = VertexLerp(isoLevel, grid, 3, 7);

        // Create the triangles.
        for(int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
            const MeshGeomVertex &v0 = vertList[triTable[cubeIndex][i+0]];
            const MeshGeomVertex &v1 = vertList[triTable[cubeIndex][i+1]];
            const MeshGeomVertex &v2 = vertList[triTable[cubeIndex][i+2]];

            MeshGeomVertex &hv0 = vHash.HashVertex(v0, geom.vertices);
            int i0 = int(&hv0 - &geom.vertices[0]);

            MeshGeomVertex &hv1 = vHash.HashVertex(v1, geom.vertices);
            int i1 = int(&hv1 - &geom.vertices[0]);

            MeshGeomVertex &hv2 = vHash.HashVertex(v2, geom.vertices);
            int i2 = int(&hv2 - &geom.vertices[0]);

            geom.indices.append(i0);
            geom.indices.append(i1);
            geom.indices.append(i2);
        }
    }

}

void MarchingCubes::compute_surface(MeshGeom &geom, const Bone_hrbf *bone)
{
    const HermiteRBF &hrbf = bone->get_hrbf();
    BBox_cu bbox = bone->get_bbox();

    // set the size of the grid cells, and the amount of cells per side
    int gridRes = 16;

    float deltaX = (bbox.pmax.x - bbox.pmin.x) / gridRes;
    float deltaY = (bbox.pmax.y - bbox.pmin.y) / gridRes;
    float deltaZ = (bbox.pmax.z - bbox.pmin.z) / gridRes;

    VertexHash vHash;
    for(float px = bbox.pmin.x; px < bbox.pmax.x; px += deltaX) {
        for(float py = bbox.pmin.y; py < bbox.pmax.y; py += deltaY) {
            for(float pz = bbox.pmin.z; pz < bbox.pmax.z; pz += deltaZ) {
                GridCell cell;
                cell.p[0].x = px;            cell.p[0].y = py;            cell.p[0].z = pz;
                cell.p[1].x = px + deltaX;   cell.p[1].y = py;            cell.p[1].z = pz;
                cell.p[2].x = px + deltaX;   cell.p[2].y = py + deltaY;   cell.p[2].z = pz;
                cell.p[3].x = px;            cell.p[3].y = py + deltaY;   cell.p[3].z = pz;
                cell.p[4].x = px;            cell.p[4].y = py;            cell.p[4].z = pz + deltaZ;
                cell.p[5].x = px + deltaX;   cell.p[5].y = py;            cell.p[5].z = pz + deltaZ;
                cell.p[6].x = px + deltaX;   cell.p[6].y = py + deltaY;   cell.p[6].z = pz + deltaZ;
                cell.p[7].x = px;            cell.p[7].y = py + deltaY;   cell.p[7].z = pz + deltaZ;

                for(int i = 0; i < 8; i++)
                {
                    const MPoint &c = cell.p[i];
                    Vec3_cu gf;
                    // XXX: bring back hrbf.f()? we don't need the gradient
                    cell.val[i] = hrbf.fngf(gf, Point_cu((float) c.x, (float) c.y, (float) c.z));
                }

                // generate triangles from this cell
                polygonize(cell, vHash, geom);
            }
        }
    }
}
MObject MarchingCubes::create_visualization_geom(const MeshGeom &srcGeom, MStatus *status)
{
    int numPolygons = srcGeom.indices.length() / 3;
    
    int numVertices = (int)srcGeom.vertices.size();
    MPointArray vertexArray(numVertices);
    for(int i = 0; i < numVertices; i++)
        vertexArray[i] = srcGeom.vertices[i].pos;

    MIntArray polygonCounts(numPolygons, 3);
    MIntArray polygonConnects(srcGeom.indices);

    MFnMeshData meshData;
    MObject fnMeshObj = meshData.create();

    MFnMesh fnMesh;
    fnMesh.create(numVertices, numPolygons, vertexArray, polygonCounts, polygonConnects, fnMeshObj, status);
    if(*status != MS::kSuccess) return MObject();

    MIntArray vertexIndices(numVertices);
    MColorArray colors(numVertices);
    for(int i = 0; i < numVertices; i++) {
        vertexIndices[i] = i;
        colors[i] = srcGeom.vertices[i].col;
    }
    fnMesh.setVertexColors(colors, vertexIndices);
    fnMesh.lockVertexNormals(vertexIndices);
    return fnMeshObj;
}
