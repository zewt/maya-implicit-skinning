#ifndef IMPLICIT_SURFACE_HPP
#define IMPLICIT_SURFACE_HPP

#include "maya_helpers.hpp"
#include "sample_set.hpp"

#include <maya/MPxDeformerNode.h> 
#include <maya/MPxSurfaceShape.h>
#include <maya/MPxSurfaceShapeUI.h>
#include <maya/MPxComponentShape.h>
#include <maya/MPxGeometryOverride.h>
#include <maya/MGeometry.h>

#include <memory>

class Bone;

#include "implicit_surface_geometry_override.hpp"

class ImplicitSurface: public MPxSurfaceShape, public ImplicitSurfaceGeometryOverrideSource
{
public:
    static void *creator();
    static MStatus initialize();
    static DagHelpers::MayaDependencies dependencies;

    static MTypeId id;

    static MObject hrbfRadiusAttr;
    static MObject samplePointAttr;
    static MObject sampleNormalAttr;

    // The bone direction at the time we were created. XXX: see if this is really needed
    static MObject initialDir;

    // Our main ImplicitBone output.  The skeleton is in world space.
    static MObject worldImplicit;

    void postConstructor();

    MStatus setDependentsDirty(const MPlug &plug, MPlugArray &plugArray);
    bool setInternalValueInContext(const MPlug &plug, const MDataHandle &dataHandle, MDGContext &ctx);

    bool isBounded() const;
    MBoundingBox boundingBox() const;
    MStatus compute(const MPlug &plug, MDataBlock &dataBlock);

    MStatus save_sampleset(const SampleSet::InputSample &inputSample);

    const MeshGeom &get_mesh_geometry();

    // This is only used during creation.
    MStatus set_bone_direction(Vec3_cu dir);
//    const Bone &get_bone() const { return bone; }

    // The mode used to blend this joint with its children.
    static MObject blendMode;

    // The bulge strength, used only when blendMode is set to BULGE.
    static MObject bulgeStrength;
    
private:
    // compute() implementations:
    MStatus load_sampleset(MDataBlock &dataBlock);
    MStatus load_mesh_geometry(MDataBlock &dataBlock);
    MStatus load_world_implicit(const MPlug &plug, MDataBlock &dataBlock);

    void set_world_space(Transfo tr);

    // The bone that we represent.  This is our primary data.
    std::shared_ptr<Bone> bone;

    // A skeleton containing just our bone.
    std::shared_ptr<Skeleton> boneSkeleton;

    // This is updated by meshGeometryUpdateAttr, and contains a mesh reprensentation of the
    // implicit surface.  This is used for preview rendering.  If the surface shape is hidden
    // (which is normally is, when being used for deformation), this won't be evaluated.
    MeshGeom meshGeometry;

    // Internal dependency attributes:
    // Evaluated when we need to update a SampleSet and load it:
    static MObject sampleSetUpdateAttr;

    // This is marked dirty to tell ImplicitSurfaceGeometryOverride that it needs to
    // recalculate the displayed geometry.
    static MObject meshGeometryUpdateAttr;
};

class ImplicitSurfaceUI: public MPxSurfaceShapeUI
{
public:
    static void *creator() { return new ImplicitSurfaceUI; }

    ImplicitSurfaceUI() { }

private:
    ImplicitSurfaceUI(const ImplicitSurfaceUI &obj);
    const ImplicitSurfaceUI &operator=(const ImplicitSurfaceUI &obj);
};

#endif
