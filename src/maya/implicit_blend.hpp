#ifndef IMPLICIT_BLEND_HPP
#define IMPLICIT_BLEND_HPP

#include "skeleton.hpp"
#include "mesh.hpp"
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
#include "implicit_surface_geometry_override.hpp"

class ImplicitBlend: public MPxSurfaceShape, public ImplicitSurfaceGeometryOverrideSource
{
public:
    static void *creator();
    static MStatus initialize();
    static DagHelpers::MayaDependencies dependencies;

    static MTypeId id;

    bool isBounded() const;
    MBoundingBox boundingBox() const;
    MStatus setDependentsDirty(const MPlug &plug_, MPlugArray &plugArray);
    MStatus compute(const MPlug &plug, MDataBlock &dataBlock);
    const MeshGeom &get_mesh_geometry();

    static MObject surfaces;

    // The parent index of each joint.  This is relative to other entries in influenceJointsAttr.  Root
    // joints are set to -1.
    static MObject parentJoint;

    static MObject implicit;
    static MObject worldImplicit;

    // The ISO used for the preview display (default 0.5).
    static MObject previewIso;

private:
    // compute() implementations:
    MStatus load_world_implicit(const MPlug &plug, MDataBlock &dataBlock);
    MStatus load_mesh_geometry(MDataBlock &dataBlock);
    MStatus get_input_bones(MDataBlock &dataBlock, std::vector<shared_ptr<const Bone> > &bones, std::vector<Bone::Id> &parents) const;
    MStatus update_skeleton(MDataBlock &dataBlock);
    MStatus update_skeleton_params(MDataBlock &dataBlock);

    std::shared_ptr<Skeleton> skeleton;

    // These are stored by load_world_implicit to tell if any values have changed since the
    // last update.
    vector<shared_ptr<const Bone> > lastImplicitBones;
    vector<int> lastParents;

    // This is updated by meshGeometryUpdateAttr, and contains a mesh reprensentation of the
    // implicit surface.  This is used for preview rendering.  If the surface shape is hidden
    // (which is normally is, when being used for deformation), this won't be evaluated.
    MeshGeom meshGeometry;

    // This is marked dirty to tell ImplicitSurfaceGeometryOverride that it needs to
    // recalculate the displayed geometry.
    static MObject meshGeometryUpdateAttr;
};

#endif
