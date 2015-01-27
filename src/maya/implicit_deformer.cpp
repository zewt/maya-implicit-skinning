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
#include <maya/MFnEnumAttribute.h>

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
MObject ImplicitDeformer::deformerIterations;
MObject ImplicitDeformer::iterativeSmoothing;
MObject ImplicitDeformer::finalFitting;
MObject ImplicitDeformer::finalSmoothingMode;

DagHelpers::MayaDependencies ImplicitDeformer::dependencies;

MStatus ImplicitDeformer::initialize()
{
    return handle_exceptions([&] {
        MStatus status = MStatus::kSuccess;

        DagHelpers::MayaDependencies dep;

        // XXX
        // MGlobal::executeCommand("makePaintable -attrType multiFloat -sm deformer blendNode weights;");

        MFnMatrixAttribute mAttr;
        MFnNumericAttribute numAttr;
        MFnCompoundAttribute cmpAttr;
        MFnTypedAttribute typedAttr;
        MFnEnumAttribute enumAttr;
    
        implicit = typedAttr.create("implicit", "implicit", ImplicitSurfaceData::id, MObject::kNullObj, &status); merr("typedAttr.create(implicit)");
        typedAttr.setReadable(false);
        addAttribute(implicit);

        deformerIterations = numAttr.create("deformerIterations", "deformerIterations", MFnNumericData::Type::kInt, 250, &status);
        numAttr.setMin(0);
        numAttr.setSoftMax(500);
        addAttribute(deformerIterations);
        dependencies.add(deformerIterations, outputGeom);

        iterativeSmoothing = numAttr.create("iterativeSmoothing", "iterativeSmoothing", MFnNumericData::Type::kBoolean, true, &status);
        addAttribute(iterativeSmoothing);
        dependencies.add(ImplicitDeformer::iterativeSmoothing, ImplicitDeformer::outputGeom);

        finalFitting = numAttr.create("finalFitting", "finalFitting", MFnNumericData::Type::kBoolean, true, &status);
        addAttribute(finalFitting);
        dependencies.add(ImplicitDeformer::finalFitting, ImplicitDeformer::outputGeom);
    
        // Don't use the raw values of EAnimesh::Smooth_type here.  Maya saves the integer value
        // to the file for some reason (it should save the string), and we shouldn't embed the
        // order of Animesh's enums into the file format.
        finalSmoothingMode = enumAttr.create("finalSmoothingMode", "finalSmoothingMode", 0, &status);
        enumAttr.addField("Laplacian", 0);
        enumAttr.addField("Conservative", 1);
        enumAttr.addField("Tangental", 2);
        enumAttr.addField("Humphrey", 3);
        enumAttr.addField("Disabled", 4);
        addAttribute(finalSmoothingMode);
        dependencies.add(ImplicitDeformer::finalSmoothingMode, ImplicitDeformer::outputGeom);

        // The base potential of the mesh.
        basePotential = numAttr.create("basePotential", "bp", MFnNumericData::Type::kFloat, 0, &status);
        numAttr.setArray(true);
        numAttr.setUsesArrayDataBuilder(true);
        addAttribute(basePotential);
        dependencies.add(implicit, basePotential);

        dependencies.add(ImplicitDeformer::implicit, ImplicitDeformer::outputGeom);
        dependencies.add(ImplicitDeformer::basePotential, ImplicitDeformer::outputGeom);
        dependencies.add(ImplicitDeformer::input, ImplicitDeformer::outputGeom);
        dependencies.add(ImplicitDeformer::inputGeom, ImplicitDeformer::outputGeom);

        status = dependencies.apply(); merr("dependencies.apply");
    });
}

void ImplicitDeformer::postConstructor()
{
    implicitIsConnected = false;
}

bool ImplicitDeformer::setInternalValueInContext(const MPlug &plug, const MDataHandle &dataHandle, MDGContext &ctx)
{
    return handle_exceptions_ret<bool>(false, [&] {
        MStatus status = MStatus::kSuccess;
        if(plug == basePotential)
        {
            bool result = MPxDeformerNode::setInternalValueInContext(plug, dataHandle, ctx);

            // load_mesh should update this, but it won't if it short-circuits due to the
            // !skeletonChanged optimization.
            MDataBlock dataBlock = forceCache();
            load_base_potential(dataBlock);
        
            return result;
        }

        return MPxDeformerNode::setInternalValueInContext(plug, dataHandle, ctx);
    });
}

// Remember whether implicit inputs are connected, since Maya doesn't clear them on
// disconnection.
MStatus ImplicitDeformer::connectionMade(const MPlug &plug, const MPlug &otherPlug, bool asSrc)
{
    return handle_exceptions_ret([&] {
        if(!asSrc && plug == ImplicitDeformer::implicit)
            implicitIsConnected = true;

        return MPxDeformerNode::connectionMade(plug, otherPlug, asSrc);
    });
}

MStatus ImplicitDeformer::connectionBroken(const MPlug &plug, const MPlug &otherPlug, bool asSrc)
{
    return handle_exceptions_ret([&] {
        if(!asSrc && plug == ImplicitDeformer::implicit)
            implicitIsConnected = false;

        return MPxDeformerNode::connectionBroken(plug, otherPlug, asSrc);
    });
}

MStatus ImplicitDeformer::compute(const MPlug& plug, MDataBlock& dataBlock)
{
    return handle_exceptions_ret([&] {
        // If we're calculating the output geometry, use the default implementation, which will
        // call deform().
        printf("Compute: %s\n", plug.name().asChar());
        if(plug.attribute() == ImplicitDeformer::outputGeom) return MPxDeformerNode::compute(plug, dataBlock);
    
        else return MStatus(MStatus::kUnknownParameter);
    });
}

MStatus ImplicitDeformer::deform(MDataBlock &dataBlock, MItGeometry &geomIter, const MMatrix &mat, unsigned int multiIndex)
{
    return handle_exceptions([&] {

    // We only support a single input, like skinCluster.
    if(multiIndex > 0)
        return;

    MStatus status = MStatus::kSuccess;

    if(!implicitIsConnected)
        return;

    // Read the dependency attributes that represent data we need.  We don't actually use the
    // results of inputvalue(); this is triggering updates for cudaCtrl data.
    dataBlock.inputValue(ImplicitDeformer::implicit, &status); merr("ImplicitDeformer::implicit");
    load_mesh(dataBlock);

    // If we don't have a mesh yet, stop.
    if(animesh.get() == NULL)
        return;

    // Run the algorithm.  XXX: If we're being applied to a set, use init_vert_to_fit to only
    // process the vertices we need to.
    int iterations = DagHelpers::readHandle<int>(dataBlock, ImplicitDeformer::deformerIterations, &status); merr("deformerIterations");
    animesh->set_nb_transform_steps(iterations);

    bool iterativeSmoothing = DagHelpers::readHandle<bool>(dataBlock, ImplicitDeformer::iterativeSmoothing, &status); merr("iterativeSmoothing");
    animesh->set_smooth_mesh(iterativeSmoothing);

    bool finalFitting = DagHelpers::readHandle<bool>(dataBlock, ImplicitDeformer::finalFitting, &status); merr("finalFitting");
    animesh->set_final_fitting(finalFitting);

    int smoothMode = DagHelpers::readHandle<short>(dataBlock, ImplicitDeformer::finalSmoothingMode, &status); merr("finalSmoothingMode");
    EAnimesh::Smooth_type smoothType = EAnimesh::Smooth_type::LAPLACIAN;

    // These values must match the values in initialize().
    switch(smoothMode)
    {
    case 0: smoothType = EAnimesh::Smooth_type::LAPLACIAN; break;
    case 1: smoothType = EAnimesh::Smooth_type::CONSERVATIVE; break;
    case 2: smoothType = EAnimesh::Smooth_type::TANGENTIAL; break;
    case 3: smoothType = EAnimesh::Smooth_type::HUMPHREY; break;
    case 4: smoothType = EAnimesh::Smooth_type::NONE; break;
    }
    animesh->set_smoothing_type(smoothType);

    animesh->transform_vertices();

    vector<Point_cu> result_verts;
    animesh->get_vertices(result_verts);

    // Copy out the vertices that we were actually asked to process.
    MMatrix invMat = mat.inverse();
    for ( ; !geomIter.isDone(); geomIter.next()) {
        int vertex_index = geomIter.index();

        Point_cu v = result_verts[vertex_index];
        MPoint pt = MPoint(v.x, v.y, v.z) * invMat;
        status = geomIter.setPosition(pt, MSpace::kObject); merr("setPosition");
    }
    });
}

void ImplicitDeformer::load_mesh(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    shared_ptr<const Skeleton> skel = get_implicit_skeleton(dataBlock);
    if(skel == NULL) {
        // We don't have a surface connected.  If we have an animMesh, discard it, since it's
        // pointing to an old Skeleton that no longer exists.
        animesh.reset();
        return;
    }

    // Get input.
    MArrayDataHandle inputArray = dataBlock.inputArrayValue(input, &status); merr("inputArrayValue(input)");

    // Get input[multiIndex].
    MDataHandle inputGeomData = DagHelpers::readArrayHandleLogicalIndex<MDataHandle>(inputArray, 0, &status); merr("readArrayHandleLogicalIndex(inputArray)");

    // Get input[multiIndex].inputGeometry.
    MDataHandle inputGeomDataHandle = inputGeomData.child(inputGeom);

    MObject geom = inputGeomDataHandle.asMesh();
    if(!geom.hasFn(MFn::kMesh))
        throw runtime_error("Only meshes are supported.");

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
    //
    // If our input skeleton has changed, it's guaranteed to be different from the Skeleton* pointer
    // in animMesh, because animMesh won't release its previous Skeleton.
    bool skeletonChanged = animesh.get() == NULL || animesh->get_skel() != skel.get();

    // Hack: We calculate a bunch of properties from the mesh, such as the nearest joint to each
    // vertex.  We don't want to recalculate that every time our input (skinned) geometry changes.
    // Maya only tells us that the input data has changed, not how.  For now, if we already have
    // geometry loaded and it has the same number of vertices, assume that we already have the correct
    // mesh loaded.  This will handle the mesh being disconnected, etc.  It'll fail on the edge case
    // of switching out the geometry with another mesh that has the same number of vertices but a
    // completely different topology.  XXX
    if(!skeletonChanged && animesh.get() != NULL)
    {
        MItGeometry allGeomIter(inputGeomDataHandle, true);

        MPointArray points;
        status = allGeomIter.allPositions(points, MSpace::kObject); merr("allGeomIter.allPositions");

        if(points.length() == mesh.get()->get_nb_vertices())
        {
            // Set the deformed vertex data.  Input normals are only used during sampling,
            // not during deformation, so we don't need to update them here.
            vector<Vec3_cu> inputVerts;
            inputVerts.reserve(points.length());
            for(int i = 0; i < (int) points.length(); ++i)
            {
                MPoint point = points[i] * worldMatrix;
                inputVerts.push_back(Vec3_cu((float) point.x, (float) point.y, (float) point.z));
            }

            animesh->set_vertices(inputVerts);

            return;
        }
    }

    // Load the input mesh from the unskinned geometry.
    Loader::Abs_mesh loaderMesh;

    MayaData::load_mesh(geom, loaderMesh, worldMatrix);

    // Create our Mesh from the loaderMesh, discarding any previous mesh.
    mesh.reset(new Mesh(loaderMesh));
    mesh->check_integrity();

    // Create a new animMesh with the current mesh and skeleton.
    animesh.reset(AnimeshBase::create(mesh.get(), skel));

    // Load base potential.
    load_base_potential(dataBlock);
}

// Update the base potential for the current mesh and input implicit surface.
MStatus ImplicitDeformer::calculate_base_potential()
{
    MDataBlock &dataBlock = this->forceCache();
    MStatus status = MStatus::kSuccess;

    // Make sure our dependencies are up to date.
    dataBlock.inputValue(ImplicitDeformer::implicit, &status); check("inputValue(implicit)");

    load_mesh(dataBlock);

    // If we don't have a mesh yet, don't do anything.
    if(animesh.get() == NULL)
        return MStatus::kSuccess;

    // Update base potential.
    animesh->update_base_potential();

    // Read the result.
    vector<float> pot;
    animesh->get_base_potential(pot);

    // Save the base potential to basePotential.
    status = DagHelpers::setArray(dataBlock, ImplicitDeformer::basePotential, pot); check("setArray(basePotential)");

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

std::shared_ptr<const Skeleton> ImplicitDeformer::get_implicit_skeleton(MDataBlock &dataBlock)
{
    MStatus status;
    MDataHandle implicitHandle = dataBlock.inputValue(ImplicitDeformer::implicit, &status); merr("get_implicit_skeleton(inputValue)");
    MFnPluginData fnData(implicitHandle.data(), &status); merr("fnData(implicitHandle)");
    ImplicitSurfaceData *data = (ImplicitSurfaceData *) fnData.data(&status); merr("fnData.data(implicit)");

    return data->getSkeleton();
}

void ImplicitDeformer::load_base_potential(MDataBlock &dataBlock)
{
    MStatus status = MStatus::kSuccess;

    // If we don't have the animMesh to load into yet, stop.
    if(animesh.get() == NULL)
        return;

    MArrayDataHandle basePotentialHandle = dataBlock.inputArrayValue(ImplicitDeformer::basePotential, &status); merr("basePotential");

    vector<float> pot;
    status = DagHelpers::readArray(basePotentialHandle, pot); merr("readArray(basePotential)");

    // Set the base potential that we loaded.
    animesh->set_base_potential(pot);
}
