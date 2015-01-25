#include "marching_cubes.hpp"
#include "tables.h"
#include "timer.hpp"
#include "cuda_current_device.hpp"
#include "skeleton_env_evaluator.hpp"

namespace MarchingCubes
{
    struct GridCell {
        Point_cu p[8];
        Vec3_cu normal[8];
        Point_cu c[8];
        float val[8];
    };

    MeshGeomVertex VertexLerp(const float isoLevel, const GridCell& cell, const int i1, const int i2) {
        // Linearly interpolate the position where an isosurface cuts
        // an edge between two vertices, each with their own scalar value
        MeshGeomVertex res;
        if(fabsf(isoLevel - cell.val[i1]) < 0.00001f) {
            res.pos = cell.p[i1];
            res.col = cell.c[i1];
            res.normal = cell.normal[i1];
            return res;
        }
        if(fabsf(isoLevel - cell.val[i2]) < 0.00001f) {
            res.pos = cell.p[i2];
            res.col = cell.c[i2];
            res.normal = cell.normal[i2];
            return res;
        }
    
        if(fabsf(cell.val[i1] - cell.val[i2]) < 0.00001f) {
            res.pos = cell.p[i1];
            res.col = cell.c[i1];
            res.normal = cell.normal[i1];
            return res;
        }

        float mu = (isoLevel - cell.val[i1]) / (cell.val[i2] - cell.val[i1]);

        res.pos = cell.p[i1] + (cell.p[i2] - cell.p[i1])*mu;
        res.col = cell.c[i1] + (cell.c[i2] - cell.c[i1])*mu;
        res.normal = cell.normal[i1] + (cell.normal[i2] - cell.normal[i1])*mu;

        return res;
    }

    void polygonize(const GridCell &grid, MeshGeom &geom, float isoLevel)
    {
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

#include "cuda_utils_common.hpp"

template<typename T>
class CudaManagedArray
{
public:
    T *p;
    const int size;

    CudaManagedArray(int size_):
        size(size_)
    {
        CUDA_SAFE_CALL(cudaMallocManaged(&p, sizeof(T) * size));

        for(int i = 0; i < size; ++i)
            new(&p[i]) T();
    }

    ~CudaManagedArray()
    {
        for(int i = 0; i < size; ++i)
            p[i].~T();
        cudaFree(p);
    }

    T &operator[](int n) { return p[n]; }
    const T &operator[](int n) const { return p[n]; }
};

namespace
{
    __global__
    void compute_marching_cubes_grid(int skel_id, int gridRes, float *isoBuffer, Vec3_cu *normals,
        Point_cu origin, Point_cu delta, Transfo transfo)
    {
        int idx = blockIdx.x * blockDim.x + threadIdx.x;
        if(idx >= gridRes*gridRes*gridRes)
            return;

        int x = idx / (gridRes*gridRes);
        int y = (idx / gridRes) % gridRes;
        int z = idx % gridRes;

        Point_cu pWorld = Point_cu(x, y, z)*delta;
        pWorld = pWorld  + origin;
        pWorld = transfo * pWorld;

        isoBuffer[idx] = Skeleton_env::compute_potential(skel_id, pWorld, normals[idx]);
    }
}

void MarchingCubes::compute_surface(MeshGeom &geom, const Skeleton *skel, float isoLevel)
{
    // Get the set of all of the bounding boxes in the skeleton.  These may overlap.

    // set the size of the grid cells, and the amount of cells per side
    const int gridRes = 16;

    const Point_cu deltas[8] = {
        Point_cu(0, 0, 0),
        Point_cu(1, 0, 0),
        Point_cu(1, 1, 0),
        Point_cu(0, 1, 0),
        Point_cu(0, 0, 1),
        Point_cu(1, 0, 1),
        Point_cu(1, 1, 1),
        Point_cu(0, 1, 1),
    };

    // We have a single surface comprised of any number of bones.  Render the bounding box
    // of each underlying bone.  Note that we only render the skeleton, not the bones.
    for(Bone::Id bone_id: skel->get_bone_ids())
    {
        const Bone *bone = skel->get_bone(bone_id).get();

        // The surface bounding box is the bbox at iso 0.5.  If we're showing ISO 0.5 or greater,
        // use that, since it's much smaller, giving us better resolution for a given grid resolution.
        // If the user has changed the ISO level to display the surface beyond that, use the full
        // bbox.
        bool use_surface_bbox = isoLevel >= 0.5 - 1e-6;
        OBBox_cu obbox = bone->get_obbox(use_surface_bbox);

        // Don't draw a grid for empty regions.
        BBox_cu &bb = obbox._bb;
        if(!bb.is_valid())
            continue;

        // We've been given the bounding box, eg. where the iso == 0.5 at both ends.  We want
        // to scan just beyond that, where iso < 0.5.  We won't draw all of the boundaries if
        // we never actually cross 0.5.  Extend the grid slightly in all directions.  Additionally,
        // blended surfaces may extend beyond the bounding box of any of the underlying bones,
        // so we extend the bbox a little further.
        Vec3_cu box_size = bb.pmax - bb.pmin;
        bb.pmin = bb.pmin - box_size * 0.2f;
        bb.pmax = bb.pmax + box_size * 0.2f;

        Point_cu delta = (obbox._bb.pmax - obbox._bb.pmin).to_point() / gridRes;

        // Calculate the iso and normal at each grid position.
        CudaManagedArray<float> isoBuffer(gridRes*gridRes*gridRes);
        CudaManagedArray<Vec3_cu> normalBuffer(gridRes*gridRes*gridRes);

        const int block_size = 512;
        const int grid_size = (gridRes*gridRes*gridRes) / block_size;
        CUDA_CHECK_KERNEL_SIZE(block_size, grid_size);
        compute_marching_cubes_grid<<<grid_size, block_size>>>(skel->get_skel_id(), gridRes, &isoBuffer[0], &normalBuffer[0], obbox._bb.pmin, delta, obbox._tr);
        CUDA_CHECK_ERRORS();

        for(int x = 0; x < gridRes-1; ++x) {
            for(int y = 0; y < gridRes-1; ++y) {
                for(int z = 0; z < gridRes-1; ++z) {
                    GridCell cell;
                    for(int i = 0; i < 8; i++)
                    {
                        int pX = x+deltas[i].x;
                        int pY = y+deltas[i].y;
                        int pZ = z+deltas[i].z;

                        // (pX,pY,pZ) is the grid position, eg. [0,15].  Multiply by delta to scale to
                        // the axis-aligned bounding box, add pmin to offset to the bounding box, and
                        // multiply by _tr to convert to world space.
                        Point_cu pWorld = Point_cu(pX, pY, pZ)*delta;
                        pWorld = pWorld  + obbox._bb.pmin;
                        pWorld = obbox._tr * pWorld;

                        cell.p[i] = pWorld;

                        int idx =
                            pX*gridRes*gridRes +
                            pY*gridRes +
                            pZ;

                        cell.val[i] = isoBuffer[idx];

                        // Gradients point in towards the surface.  Multiply by -1 to get a normal pointing
                        // away from the surface.
                        cell.normal[i] = normalBuffer[idx] * -1;
                    }

                    // Generate tris.
                    polygonize(cell, geom, isoLevel);
                }
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
