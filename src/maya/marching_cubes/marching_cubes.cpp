#define NO_CUDA

#include "marching_cubes.hpp"
#include "tables.h"

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

    void polygonize(const GridCell &grid, MeshGeom &geom)
    {
        const float isoLevel = 0.5f;

        // Determine the index into the edge table which
        // tells us which vertices are inside of the surface
        int cubeIndex = 0;
        for(int i = 0; i < 8; ++i)
            if(grid.val[i] < isoLevel) cubeIndex |= 1<<i;

        // Cube is entirely in/out of the surface
        if(edgeTable[cubeIndex] == 0)
            return;

        // Find the vertices where the surface intersects the cube
        MeshGeomVertex vertList[12];
        if(edgeTable[cubeIndex] & (1<<0))   vertList[0] = VertexLerp(isoLevel, grid, 0, 1);
        if(edgeTable[cubeIndex] & (1<<1))   vertList[1] = VertexLerp(isoLevel, grid, 1, 2);
        if(edgeTable[cubeIndex] & (1<<2))   vertList[2] = VertexLerp(isoLevel, grid, 2, 3);
        if(edgeTable[cubeIndex] & (1<<3))   vertList[3] = VertexLerp(isoLevel, grid, 3, 0);
        if(edgeTable[cubeIndex] & (1<<4))   vertList[4] = VertexLerp(isoLevel, grid, 4, 5);
        if(edgeTable[cubeIndex] & (1<<5))   vertList[5] = VertexLerp(isoLevel, grid, 5, 6);
        if(edgeTable[cubeIndex] & (1<<6))   vertList[6] = VertexLerp(isoLevel, grid, 6, 7);
        if(edgeTable[cubeIndex] & (1<<7))   vertList[7] = VertexLerp(isoLevel, grid, 7, 4);
        if(edgeTable[cubeIndex] & (1<<8))   vertList[8] = VertexLerp(isoLevel, grid, 0, 4);
        if(edgeTable[cubeIndex] & (1<<9))   vertList[9] = VertexLerp(isoLevel, grid, 1, 5);
        if(edgeTable[cubeIndex] & (1<<10)) vertList[10] = VertexLerp(isoLevel, grid, 2, 6);
        if(edgeTable[cubeIndex] & (1<<11)) vertList[11] = VertexLerp(isoLevel, grid, 3, 7);

        // Create the triangles.
        for(int i = 0; triTable[cubeIndex][i] != -1; i += 3) {
            geom.vertices.push_back(vertList[triTable[cubeIndex][i+0]]);
            geom.vertices.push_back(vertList[triTable[cubeIndex][i+1]]);
            geom.vertices.push_back(vertList[triTable[cubeIndex][i+2]]);
        }
    }

}

void MarchingCubes::compute_surface(MeshGeom &geom, const Bone *bone)
{
    const HermiteRBF &hrbf = bone->get_hrbf();
    BBox_cu bbox = bone->get_bbox();

    // set the size of the grid cells, and the amount of cells per side
    int gridRes = 16;

    float deltaX = (bbox.pmax.x - bbox.pmin.x) / gridRes;
    float deltaY = (bbox.pmax.y - bbox.pmin.y) / gridRes;
    float deltaZ = (bbox.pmax.z - bbox.pmin.z) / gridRes;

    MFloatPoint deltas[8];
    deltas[0] = MFloatPoint(0,      0,      0);
    deltas[1] = MFloatPoint(deltaX, 0,      0);
    deltas[2] = MFloatPoint(deltaX, deltaY, 0);
    deltas[3] = MFloatPoint(0,      deltaY, 0);
    deltas[4] = MFloatPoint(0,      0,      deltaZ);
    deltas[5] = MFloatPoint(deltaX, 0,      deltaZ);
    deltas[6] = MFloatPoint(deltaX, deltaY, deltaZ);
    deltas[7] = MFloatPoint(0,      deltaY, deltaZ);

    for(float px = bbox.pmin.x; px < bbox.pmax.x; px += deltaX) {
        for(float py = bbox.pmin.y; py < bbox.pmax.y; py += deltaY) {
            for(float pz = bbox.pmin.z; pz < bbox.pmax.z; pz += deltaZ) {
                MFloatPoint p(px, py, pz);

                GridCell cell;
                for(int i = 0; i < 8; i++)
                {
                    MFloatPoint &c = cell.p[i];
                    c = p + deltas[i];

                    Vec3_cu gf;
                    // XXX: bring back hrbf.f()? we don't need the gradient
                    cell.val[i] = hrbf.fngf(gf, Point_cu((float) c.x, (float) c.y, (float) c.z));
                }

                // generate triangles from this cell
                polygonize(cell, geom);
            }
        }
    }
}

/* 
    ================================================================================
    Copyright (c) 2010, Jose Esteve. http://www.joesfer.com
    All rights reserved. 

    Redistribution and use in source and binary forms, with or without modification, 
    are permitted provided that the following conditions are met: 

    * Redistributions of source code must retain the above copyright notice, this 
      list of conditions and the following disclaimer. 
    
    * Redistributions in binary form must reproduce the above copyright notice, 
      this list of conditions and the following disclaimer in the documentation 
      and/or other materials provided with the distribution. 
    
    * Neither the name of the organization nor the names of its contributors may 
      be used to endorse or promote products derived from this software without 
      specific prior written permission. 

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR 
    ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
    ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
    ================================================================================
*/
