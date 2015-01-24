#define NO_CUDA

#include "implicit_deformer.hpp"
#include "implicit_surface_data.hpp"

#include <string.h>
#include <math.h>
#include <assert.h>

#include <maya/MGlobal.h> 
#include <maya/MPxData.h>
#include <maya/MItGeometry.h>
#include <maya/MItMeshVertex.h>
#include <maya/MArrayDataBuilder.h>

#include <maya/MFnNumericAttribute.h>
#include <maya/MFnCompoundAttribute.h>
#include <maya/MFnMatrixAttribute.h>
#include <maya/MFnTypedAttribute.h>
#include <maya/MFnMatrixData.h>

#include <maya/MFnPluginData.h>

#include <maya/MTypeId.h> 
#include <maya/MPlug.h>
#include <maya/MFnMesh.h>

#include <maya/MDataBlock.h>
#include <maya/MDataHandle.h>
#include <maya/MPointArray.h>
#include <maya/MMatrix.h>

#include "maya/maya_helpers.hpp"
#include "maya/maya_data.hpp"

#include "skeleton.hpp"
#include "animated_mesh_ctrl.hpp"

#include <algorithm>
#include <map>
using namespace std;



// XXX: Maya error handling is hard to work with.  There are no exceptions, so we have to
// check every single call manually.  It's also not possible to create your own MStatus error
// messages, which means our own error handling can't be done consistently with Maya's.  Maybe
// we should wrap the top-level entry points in an exception handler, so we can throw.  We'd
// still need to sprinkle error checking everywhere, though.
#define check(s) \
    { \
        if(status != MS::kSuccess) { \
            status.perror(s); \
            return status; \
        } \
    }




// XXX: http://help.autodesk.com/view/MAYAUL/2015/ENU/?guid=__cpp_ref_class_m_type_id_html says that
// ADN assigns public blocks of IDs, but nothing says how to request a block without paying for
// a commercial ADN account.  Let's use a value in the devkit sample range, so it's unlikely to conflict,
// and it does, it won't conflict with somebody's internal-use IDs (0-0x7ffff).  At worst, we'll collide
// with a sample or somebody else doing the same thing.
const MTypeId ImplicitDeformer::id(0xEA115);
MObject ImplicitDeformer::implicit;
MObject ImplicitDeformer::basePotential;
MObject ImplicitDeformer::baseGradient;

DagHelpers::MayaDependencies ImplicitDeformer::dependencies;

MStatus ImplicitDeformer::initialize()
{
    MStatus status = MStatus::kSuccess;

    DagHelpers::MayaDependencies dep;

    // XXX
    // MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer blendNode weights;");

    MFnMatrixAttribute mAttr;
    MFnNumericAttribute numAttr;
    MFnCompoundAttribute cmpAttr;
    MFnTypedAttribute typedAttr;
    
    implicit = typedAttr.create("implicit", "implicit", ImplicitSurfaceData::id, MObject::kNullObj, &status);
    if(status != MS::kSuccess) return status;
    typedAttr.setHidden(true);
    addAttribute(implicit);

    // The base potential of the mesh.
    basePotential = numAttr.create("basePotential", "bp", MFnNumericData::Type::kFloat, 0, &status);
    numAttr.setArray(true);
//    numAttr.setInternal(true);
    numAttr.setUsesArrayDataBuilder(true);
    addAttribute(basePotential);
    dependencies.add(implicit, basePotential);
    

    baseGradient = numAttr.create("baseGradient", "bg", MFnNumericData::Type::k3Float, 0, &status);
    numAttr.setArray(true);
//    numAttr.setInternal(true);
    numAttr.setUsesArrayDataBuilder(true);
    addAttribute(baseGradient);
    dependencies.add(implicit, baseGradient);

    dependencies.add(ImplicitDeformer::implicit, ImplicitDeformer::outputGeom);
    dependencies.add(ImplicitDeformer::basePotential, ImplicitDeformer::outputGeom);
    dependencies.add(ImplicitDeformer::baseGradient, ImplicitDeformer::outputGeom);
    dependencies.add(ImplicitDeformer::input, ImplicitDeformer::outputGeom);
    dependencies.add(ImplicitDeformer::inputGeom, ImplicitDeformer::outputGeom);

    status = dependencies.apply();
    if(status != MS::kSuccess) return status;

    return MStatus::kSuccess;
}

void ImplicitDeformer::postConstructor()
{
}

bool ImplicitDeformer::setInternalValueInContext(const MPlug &plug, const MDataHandle &dataHandle, MDGContext &ctx)
{
    MStatus status = MStatus::kSuccess;
    if(plug == basePotential || plug == baseGradient)
    {
        bool result = MPxDeformerNode::setInternalValueInContext(plug, dataHandle, ctx);

        // load_mesh should update this, but it won't if it short-circuits due to the
        // !skeletonChanged optimization.
        MDataBlock dataBlock = forceCache();
        status = load_base_potential(dataBlock);
        if(status != MS::kSuccess) return status;
        
        return result;
    }

    return MPxDeformerNode::setInternalValueInContext(plug, dataHandle, ctx);
}

MStatus ImplicitDeformer::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    // If we're calculating the output geometry, use the default implementation, which will
    // call deform().
    printf("Compute: %s\n", plug.name().asChar());
    if(plug.attribute() == ImplicitDeformer::outputGeom) return MPxDeformerNode::compute(plug, dataBlock);
    
    else return MStatus::kUnknownParameter;
}

MStatus ImplicitDeformer::deform(MDataBlock &dataBlock, MItGeometry &geomIter, const MMatrix &mat, unsigned int multiIndex)
{
    // We only support a single input, like skinCluster.
    if(multiIndex > 0)
        return MStatus::kSuccess;

    MStatus status = MStatus::kSuccess;

    // Read the dependency attributes that represent data we need.  We don't actually use the
    // results of inputvalue(); this is triggering updates for cudaCtrl data.
    dataBlock.inputValue(ImplicitDeformer::implicit, &status);
    if(status != MS::kSuccess) return status;

    status = load_mesh(dataBlock);
    if(status != MS::kSuccess) return status;

    // If we don't have a mesh yet, stop.
    if(animMesh.get() == NULL)
        return MStatus::kSuccess;

    // Run the algorithm.  XXX: If we're being applied to a set, use init_vert_to_fit to only
    // process the vertices we need to.
    animMesh->set_do_smoothing(true);
    animMesh->deform_mesh();

    vector<Point_cu> result_verts;
    animMesh->get_anim_vertices_aifo(result_verts);

    // Copy out the vertices that we were actually asked to process.
    MMatrix invMat = mat.inverse();
    for ( ; !geomIter.isDone(); geomIter.next()) {
        int vertex_index = geomIter.index();

        Point_cu v = result_verts[vertex_index];
        MPoint pt = MPoint(v.x, v.y, v.z) * invMat;
        status = geomIter.setPosition(pt, MSpace::kObject);
        if(status != MS::kSuccess) return status;
    }

    return MStatus::kSuccess;
}

MStatus ImplicitDeformer::load_mesh(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    shared_ptr<const Skeleton> skel = get_implicit_skeleton(dataBlock, &status);
    if(skel == NULL) {
        // We don't have a surface connected.  If we have an animMesh, discard it, since it's
        // pointing to an old Skeleton that no longer exists.
        animMesh.release();
        return MStatus::kSuccess;
    }

    // Get input.
    MArrayDataHandle inputArray = dataBlock.inputArrayValue(input, &status);
    if(status != MS::kSuccess) return status;

    // Get input[multiIndex].
    MDataHandle inputGeomData = DagHelpers::readArrayHandleLogicalIndex<MDataHandle>(inputArray, 0, &status);
    if(status != MS::kSuccess) return status;

    // Get input[multiIndex].inputGeometry.
    MDataHandle inputGeomDataHandle = inputGeomData.child(inputGeom);

    MObject geom = inputGeomDataHandle.asMesh();
    if(!geom.hasFn(MFn::kMesh)) {
        // XXX: only meshes are supported
        return MStatus::kFailure;
    }

    // Get our input's transformation.  We'll load the mesh in world space according to that
    // transform.  This is different from deform() because deform() gives us the matrix to use.
    MMatrix worldMatrix = inputGeomDataHandle.geometryTransformMatrix();

    // We could be dirty because the skeleton has been modified (a joint moved), or because the skeleton
    // has been changed entirely.  If the skeleton has been changed entirely then we need to recreate
    // the animMesh to give it the new skeleton, but if it's just been modified then we don't need to
    // do that.
    //
    // If we didn't already have a skeleton, or the skeleton we've been given has a different unique ID
    // than the one we had, then this is a new skeleton.
    bool skeletonChanged = animMesh.get() == NULL || animMesh->skel->get_unique_id() != skel->get_unique_id();

    // Hack: We calculate a bunch of properties from the mesh, such as the nearest joint to each
    // vertex.  We don't want to recalculate that every time our input (skinned) geometry changes.
    // Maya only tells us that the input data has changed, not how.  For now, if we already have
    // geometry loaded and it has the same number of vertices, assume that we already have the correct
    // mesh loaded.  This will handle the mesh being disconnected, etc.  It'll fail on the edge case
    // of switching out the geometry with another mesh that has the same number of vertices but a
    // completely different topology.  XXX
    if(!skeletonChanged && animMesh.get() != NULL)
    {
        MItGeometry allGeomIter(inputGeomDataHandle, true);

        MPointArray points;
        status = allGeomIter.allPositions(points, MSpace::kObject);
        if(status != MS::kSuccess) return status;

        if(points.length() == mesh.get()->get_nb_vertices())
        {
            // Set the deformed vertex data.
            vector<Vec3_cu> input_verts;
            input_verts.reserve(points.length());
            for(int i = 0; i < (int) points.length(); ++i)
            {
                MPoint point = points[i] * worldMatrix;
                input_verts.push_back(Vec3_cu((float) point.x, (float) point.y, (float) point.z));
            }
        
            animMesh->copy_vertices(input_verts);

            return MStatus::kSuccess;
        }
    }

    // Load the input mesh from the unskinned geometry.
    Loader::Abs_mesh loaderMesh;

    status = MayaData::load_mesh(geom, loaderMesh, worldMatrix);
    if(status != MS::kSuccess) return status;

    // Create our Mesh from the loaderMesh, discarding any previous mesh.
    mesh.reset(new Mesh(loaderMesh));
    mesh->check_integrity();

    // Create a new animMesh with the current mesh and skeleton.
    animMesh.reset(new Animated_mesh_ctrl(mesh.get(), skel));

    // Load base potential.
    status = load_base_potential(dataBlock);
    if(status != MS::kSuccess) return status;

    return MStatus::kSuccess;
}

// Update the base potential for the current mesh and input implicit surface.
MStatus ImplicitDeformer::calculate_base_potential()
{
    MDataBlock &dataBlock = this->forceCache();
    MStatus status = MStatus::kSuccess;

    // Make sure our dependencies are up to date.
    dataBlock.inputValue(ImplicitDeformer::implicit, &status); check("inputValue(implicit)");

    status = load_mesh(dataBlock);
    if(status != MS::kSuccess) return status;

    // If we don't have a mesh yet, don't do anything.
    if(animMesh.get() == NULL)
        return MStatus::kSuccess;

    // Update base potential.
    animMesh->update_base_potential();

    // Read the result.
    vector<float> pot;
    vector<Vec3_cu> grad;
    animMesh->get_base_potential(pot);

    // Save the base potential to basePotential and baseGradient.
    status = DagHelpers::setArray(dataBlock, ImplicitDeformer::basePotential, pot); check("setArray(basePotential)");
    status = DagHelpers::setArray(dataBlock, ImplicitDeformer::baseGradient, grad); check("setArray(baseGradient)");

    // Work around a Maya bug.  Setting an array on the dataBlock doesn't trigger dependencies.  We
    // need to set a value using an MPlug to trigger updates.
    if(pot.size() > 0)
    {
        MPlug basePotentialPlug(thisMObject(), ImplicitDeformer::basePotential);
        basePotentialPlug = basePotentialPlug.elementByLogicalIndex(0, &status);
        if(status != MS::kSuccess) return status;
        basePotentialPlug.setFloat(pot[0]);
    }

    return MStatus::kSuccess;
}

std::shared_ptr<const Skeleton> ImplicitDeformer::get_implicit_skeleton(MDataBlock &dataBlock, MStatus *status)
{
    MDataHandle implicitHandle = dataBlock.inputValue(ImplicitDeformer::implicit, status);
    if(*status != MS::kSuccess) return NULL;

    MFnPluginData fnData(implicitHandle.data(), status);
    if(*status != MS::kSuccess) return NULL;

    ImplicitSurfaceData *data = (ImplicitSurfaceData *) fnData.data(status);
    if(*status != MS::kSuccess) return NULL;

    return data->getSkeleton();
}

MStatus ImplicitDeformer::load_base_potential(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // If we don't have the animMesh to load into yet, stop.
    if(animMesh.get() == NULL)
        return MS::kSuccess;

    MArrayDataHandle basePotentialHandle = dataBlock.inputArrayValue(ImplicitDeformer::basePotential, &status);
    if(status != MS::kSuccess) return status;

    vector<float> pot;
    status = DagHelpers::readArray(basePotentialHandle, pot);

    MArrayDataHandle baseGradientHandle = dataBlock.inputArrayValue(ImplicitDeformer::baseGradient, &status);
    if(status != MS::kSuccess) return status;

    vector<Vec3_cu> grad;
    status = DagHelpers::readArray(baseGradientHandle, grad);
    if(status != MS::kSuccess) return status;

    // Set the base potential that we loaded.
    animMesh->set_base_potential(pot);

    return MStatus::kSuccess;
}
