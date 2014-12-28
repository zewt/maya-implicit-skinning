// nVidia and Autodesk are both equally incompetent: they both declare types like "short3" in their
// headers with no namespacing whatsoever, which means that CUDA and Maya can't be used together
// without jumping hoops.  This file exposes an interface for the plugin that doesn't include any
// CUDA headers.  Seriously, this is a joke.

#include "Interface.hpp"

#include "loader_mesh.hpp"

#include "blending_lib/generator.hpp"
#include "vec3_cu.hpp"
#include "cuda_ctrl.hpp"
#include "conversions.hpp"
#include "cuda_ctrl.hpp"
#include "skeleton.hpp"
#include "animesh.hpp"

#include <vector>
using namespace std;

struct PluginInterfaceImpl
{
    bool gotFirst;
    Cuda_ctrl::CudaCtrl cudaCtrl;
};

PluginInterface::PluginInterface()
{
    // We store this in a helper object to avoid importing CudaCtrl into the header,
    // since it's incompatible with Maya's headers.
    impl = new PluginInterfaceImpl();
    impl->gotFirst = false;
}

PluginInterface::~PluginInterface()
{
    delete impl;
}

void PluginInterface::init()
{
    std::vector<Blending_env::Op_t> op;
    op.push_back( Blending_env::B_D  );
    op.push_back( Blending_env::U_OH );
    op.push_back( Blending_env::C_D  );

    Cuda_ctrl::cuda_start(op);

    // XXX "HACK: Because blending_env initialize to elbow too ..." What?
    IBL::Ctrl_setup shape = IBL::Shape::elbow();
}

void PluginInterface::shutdown()
{
    Cuda_ctrl::cleanup();
}


bool PluginInterface::is_setup() const
{
    return impl->gotFirst;
}

// inputGeometry will have no translation 

// We're being given a mesh to work with.  Load it into Mesh, and hand it to the CUDA
// interface.
void PluginInterface::setup(const Loader::Abs_mesh &loader_mesh, const Loader::Abs_skeleton &loader_skeleton)
{
    if(impl->gotFirst)
        return;
    impl->gotFirst = true;

    // Abs_mesh is a simple representation that doesn't touch CUDA.  Load it into
    // Mesh.
    Mesh *ptr_mesh = new Mesh(loader_mesh);

    // Hand the Mesh to Cuda_ctrl.
    impl->cudaCtrl.load_mesh( ptr_mesh );

    // Load the skeleton.
    impl->cudaCtrl._skeleton.load(loader_skeleton);

    // Tell cudaCtrl that we've loaded both the mesh and the skeleton.  This could be simplified.
    impl->cudaCtrl.load_animesh();

    // Run the initial sampling.  Skip bone 0, which is a dummy parent bone.
    for(int bone_id = 1; bone_id < impl->cudaCtrl._anim_mesh->_skel->nb_joints(); ++bone_id)
        impl->cudaCtrl._anim_mesh->update_hrbf_samples(bone_id, 0);

    impl->cudaCtrl._anim_mesh->update_base_potential();
}

void PluginInterface::update_skeleton(const vector<Loader::CpuTransfo> &bone_positions)
{
    // Update the skeleton representation.
    vector<Transfo> transfos;
    for(int i = 0; i < bone_positions.size(); ++i)
        transfos.push_back(Transfo(bone_positions[i]));

    impl->cudaCtrl._skeleton.set_transforms(transfos);
}

void PluginInterface::update_vertices(const vector<Loader::Vertex> &loader_vertices)
{
    size_t num_vertices = loader_vertices.size();
    vector<Vec3_cu> vertices(num_vertices);
    const Loader::Vertex *mesh_vertices = &loader_vertices[0];
    for(size_t i = 0; i < num_vertices; ++i)
        vertices[i] = Vec3_cu(mesh_vertices[i].x, mesh_vertices[i].y,mesh_vertices[i].z);
    impl->cudaCtrl._anim_mesh->_animesh->copy_vertices(vertices);
}

void PluginInterface::go(vector<Loader::Vec3> &out_verts)
{
    impl->cudaCtrl._anim_mesh->set_do_smoothing(true);
    impl->cudaCtrl._anim_mesh->deform_mesh();

    impl->cudaCtrl._anim_mesh->_animesh->get_anim_vertices_aifo(out_verts);
}
