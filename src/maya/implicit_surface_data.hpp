#ifndef IMPLICIT_SURFACE_DATA_HPP
#define IMPLICIT_SURFACE_DATA_HPP

#include <memory>

#include <maya/MPxData.h> 
#include <maya/MTypeId.h> 
#include <maya/MString.h> 

#include "skeleton.hpp"

// This node is the data type that connects ImplicitSurface, ImplicitBlend and ImplicitDeformer
// together.  It just holds a pointer to a skeleton with one or more bones.
class ImplicitSurfaceData: public MPxData
{
public:
    ImplicitSurfaceData() { }

    void copy(const MPxData &cpy_) { const ImplicitSurfaceData &cpy = (ImplicitSurfaceData &) cpy_; skel = cpy.skel; }
    MTypeId typeId() const { return id; }
    MString name() const { return typeName; }

    // Store and retrieve a pointer to the skeleton.  The skeleton may be NULL if the
    // input has no data.
    std::shared_ptr<const Skeleton> getSkeleton() const { return skel; }
    void setSkeleton(std::shared_ptr<const Skeleton> skel_) { skel = skel_; }

    static const MString typeName;
    static const MTypeId id;
    static void *creator() { return new ImplicitSurfaceData; }

private:
    std::shared_ptr<const Skeleton> skel;
};

#endif
