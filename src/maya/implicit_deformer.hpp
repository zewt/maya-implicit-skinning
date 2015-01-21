#ifndef IMPLICIT_DEFORMER_HPP
#define IMPLICIT_DEFORMER_HPP

#include "mesh.hpp"
#include "animated_mesh_ctrl.hpp"
#include "maya_helpers.hpp"

#include <maya/MPxDeformerNode.h> 

#include <memory>

class ImplicitDeformer: public MPxDeformerNode
{
public:
    static const MTypeId id;

    static void *creator() { return new ImplicitDeformer(); }
    static MStatus initialize();
    
    void postConstructor();
    MStatus compute(const MPlug& plug, MDataBlock& dataBlock);
    MStatus deform(MDataBlock &block, MItGeometry &iter, const MMatrix &mat, unsigned int multiIndex);
    bool setInternalValueInContext(const MPlug &plug, const MDataHandle &dataHandle, MDGContext &ctx);

    MStatus calculate_base_potential();

    // The base potential of the mesh, as [normalX, normalY, normalZ, pot].
    static MObject basePotential;
    static MObject baseGradient;

    // The input implicit surface.
    static MObject implicit;

private:
    static DagHelpers::MayaDependencies dependencies;

    MStatus load_mesh(MDataBlock &dataBlock);
    MStatus load_base_potential(MDataBlock &dataBlock);
    std::shared_ptr<const Skeleton> get_implicit_skeleton(MDataBlock &dataBlock, MStatus *status);

    // The loaded mesh.  We own this object.
    std::unique_ptr<Mesh> mesh;

    // The wrapper class around both the mesh and the skeleton.
    std::unique_ptr<Animated_mesh_ctrl> animMesh;
};

#endif
