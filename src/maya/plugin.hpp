#ifndef IMPLICIT_SKIN_DEFORMER_H
#define IMPLICIT_SKIN_DEFORMER_H

#ifndef NO_CUDA
#error This header requires NO_CUDA.
#endif

#include "skeleton_ctrl.hpp"
#include "mesh.hpp"
#include "animated_mesh_ctrl.hpp"
#include "bone_set.hpp"

#include <maya/MPxDeformerNode.h> 
#include <memory>
namespace SampleSet { struct SampleSet; }

#include "marching_cubes/marching_cubes.hpp"

class MeshGeom;

struct GridCell;
class VertexHash;
class Bone;

// Note that functions which don't take a dataBlock but read attributes aren't intended to be called from
// compute().  Maya has a different API for reading attributes inside compute().
class ImplicitSkinDeformer: public MPxDeformerNode
{
public:
    static const MTypeId id;

    virtual ~ImplicitSkinDeformer() { }

    static void *creator() { return new ImplicitSkinDeformer(); }
    static MStatus initialize();
    
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock);
    MStatus deform(MDataBlock &block, MItGeometry &iter, const MMatrix &mat, unsigned int multiIndex);

    MStatus save_sampleset(const SampleSet::SampleSet &samples);
    MStatus set_bind_pose();

    MStatus load_skeleton(MDataBlock &dataBlock);
    MStatus load_sampleset(MDataBlock &dataBlock);
    MStatus load_mesh(MDataBlock &dataBlock);
    MStatus load_base_potential(MDataBlock &dataBlock);
    MStatus load_visualization_geom_data(MDataBlock &dataBlock);
    MStatus load_visualization_geom(MDataBlock &dataBlock);

    MStatus sample_all_joints();
    MStatus calculate_base_potential();
    MStatus get_default_hrbf_radius(std::map<Bone::Id,float> &hrbf_radius);

    static ImplicitSkinDeformer *deformerFromPlug(MObject node, MStatus *status);

    static MObject unskinnedGeomAttr;

    // The geomMatrix of the skinCluster at setup time.
    static MObject geomMatrixAttr;

    // The base potential of the mesh, as [normalX, normalY, normalZ, pot].
    static MObject basePotentialAttr;
    static MObject baseGradientAttr;

    // Per-joint attributes:
    static MObject influenceJointsAttr;

    // The parent index of each joint.  This is relative to other entries in influenceJointsAttr.  Root
    // joints are set to -1.
    static MObject parentJointAttr;

    // Each joint's influenceBindMatrix at setup time.
    static MObject influenceBindMatrixAttr;

    // The world matrix of each influence.  This is usually linked to the worldMatrix attribute of the joint.
    static MObject influenceMatrixAttr;
    static MObject hrbfRadiusAttr;
    static MObject samplePointAttr;
    static MObject sampleNormalAttr;

    static MObject visualizationGeomUpdateAttr;
    static MObject visualizationGeomAttr;
    
    MStatus test();

    BoneSet boneSet; // XXX private
private:

    // The loaded mesh.  We own this object.
    std::auto_ptr<Mesh> mesh;

    // The loaded skeleton (if any), and its wrapper helper class.
    Skeleton_ctrl skeleton;

    // The wrapper class around both the mesh and the skeleton.
    std::auto_ptr<Animated_mesh_ctrl> animMesh;


    // The *UpdateAttr attributes are for tracking when we need to update an internal data structure to reflect
    // a change made to the Maya attributes.  These aren't saved to disk or shown to the user.  For example, when
    // we set new data samplePointAttr, it will trigger sampleSetUpdateAttr's dependency on it, causing
    // us to update the SampleSet.  Triggering the sampleSetUpdateAttr dependency will then trigger
    // basePotentialUpdateAttr, causing us to update the base potential.  This allows data to be manipulated
    // normally within Maya, and we just update our data based on it as needed.

    // Internal dependency attributes:
    // Evaluated when we need to update a SampleSet and load it:
    static MObject sampleSetUpdateAttr;

    // Represents this->skeleton being up to date:
    static MObject skeletonUpdateAttr;

    // Represents animMesh being up to date with the skinned geometry.
    static MObject meshUpdateAttr;
    
    // The contents of basePotentialAttr are loaded into animesh.
    static MObject basePotentialUpdateAttr;

    MStatus createSkeleton(MDataBlock &dataBlock, Loader::Abs_skeleton &skeleton);
    MStatus setGeometry(MDataHandle &inputGeomDataHandle);

    MObject create_visualization_geom(const MeshGeom &srcGeom, MStatus *status);
    MeshGeom previewMeshGeometry;
};

#endif
