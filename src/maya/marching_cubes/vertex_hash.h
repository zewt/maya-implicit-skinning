#ifndef vertexHash_h__
#define vertexHash_h__

#include <maya/MPoint.h>
#include "mesh_data.h"
#include <vector>
#include <unordered_map>

// Used to convert from a triangle-soup mesh into a indexed
// array of vertices. Works by hashing the space in a grid, and
// and snapping nearby vertices within a threshold
struct VertexHashValue {
    VertexHashValue(MeshGeomVertex v, float snapThreshold):
        snappedValueX(lrint(v.pos.x / snapThreshold)),
        snappedValueY(lrint(v.pos.y / snapThreshold)),
        snappedValueZ(lrint(v.pos.z / snapThreshold))
    {
    }

    bool operator==(const VertexHashValue &rhs) const {
        return snappedValueX == rhs.snappedValueX && snappedValueY == rhs.snappedValueY && snappedValueZ == rhs.snappedValueZ;
    }

    int snappedValueX, snappedValueY, snappedValueZ;
};

template<> struct std::hash<VertexHashValue>
{
    size_t operator()(const VertexHashValue &lhs) const
    {
        return lhs.snappedValueX + (lhs.snappedValueY<<8) + (lhs.snappedValueZ<<16);
    }
};

class VertexHash {
public:
    VertexHash(float snapThreshold = 0.01f);
    MeshGeomVertex &HashVertex(const MeshGeomVertex &v, std::vector<MeshGeomVertex> &vertices);

private:
    typedef std::unordered_map<VertexHashValue, size_t> mapType;
    mapType vertexIdx;
    const float snapThreshold;
};

#endif

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
