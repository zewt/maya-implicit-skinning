#ifndef IMPLICIT_BLEND_HPP
#define IMPLICIT_BLEND_HPP

#include "skeleton_ctrl.hpp"
#include "mesh.hpp"
#include "animated_mesh_ctrl.hpp"
#include "maya_helpers.hpp"
#include "sample_set.hpp"

#include <maya/MPxDeformerNode.h> 
#include <maya/MPxSurfaceShape.h>
#include <maya/MPxSurfaceShapeUI.h>
#include <maya/MPxComponentShape.h>
#include <maya/MPxGeometryOverride.h>
#include <maya/MGeometry.h>

#include <memory>

#include "implicit_surface_data.hpp"

class ImplicitBlend: public MPxSurfaceShape
{
public:
    static void *creator();
    static MStatus initialize();
    static DagHelpers::MayaDependencies dependencies;

    static MTypeId id;

//    bool isBounded() const;
//    MBoundingBox boundingBox() const;
    MStatus compute(const MPlug &plug, MDataBlock &dataBlock);


    static MObject surfaces;

    // The parent index of each joint.  This is relative to other entries in influenceJointsAttr.  Root
    // joints are set to -1.
    static MObject parentJoint;

    static MObject implicit;
    static MObject worldImplicit;

private:
    // compute() implementations:
    MStatus load_world_implicit(const MPlug &plug, MDataBlock &dataBlock);
    MStatus update_skeleton(MDataBlock &dataBlock);

    Skeleton_ctrl skeleton;

    // These are stored by load_world_implicit to tell if any values have changed since the
    // last update.
    vector<shared_ptr<const Skeleton> > lastImplicitBones;
    vector<int> lastParents;


    // This is updated by meshGeometryUpdateAttr, and contains a mesh reprensentation of the
    // implicit surface.  This is used for preview rendering.  If the surface shape is hidden
    // (which is normally is, when being used for deformation), this won't be evaluated.
//    MeshGeom meshGeometry;

    // Internal dependency attributes:
    // Evaluated when we need to update a SampleSet and load it:
    static MObject sampleSetUpdateAttr;

    // This is marked dirty to tell ImplicitSurfaceGeometryOverride that it needs to
    // recalculate the displayed geometry.
//    static MObject meshGeometryUpdateAttr;
};

#endif
