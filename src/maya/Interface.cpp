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
#include "sample_set.hpp"

#include <vector>
using namespace std;

struct PluginInterfaceImpl
{
    Cuda_ctrl::CudaCtrl cudaCtrl;
};

PluginInterface::PluginInterface()
{
    // We store this in a helper object to avoid importing CudaCtrl into the header,
    // since it's incompatible with Maya's headers.
    impl = new PluginInterfaceImpl();
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
    return impl->cudaCtrl._mesh != NULL;
}

// inputGeometry will have no translation 

// We're being given a mesh to work with.  Load it into Mesh, and hand it to the CUDA
// interface.
void PluginInterface::setup(const Loader::Abs_mesh &loader_mesh, const Loader::Abs_skeleton &loader_skeleton)
{
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
    SampleSet samples(impl->cudaCtrl._anim_mesh->_skel->nb_joints());

    // Get the default junction radius.
    impl->cudaCtrl._anim_mesh->_animesh->get_default_junction_radius(samples._junction_radius);

    for(int bone_id = 1; bone_id < impl->cudaCtrl._anim_mesh->_skel->nb_joints(); ++bone_id)
    {
        if(true)
        {
            samples.choose_hrbf_samples_poisson
                    (*impl->cudaCtrl._anim_mesh->_animesh,
                     bone_id,
                     // Set a distance threshold from sample to the joints to choose them.
                     -0.02f, // dSpinB_max_dist_joint->value(),
                     -0.02f, // dSpinB_max_dist_parent->value(),
                     0, // dSpinB_min_dist_samples->value(),
                     // Minimal number of samples.  (this value is used only whe the value min dist is zero)
                     50, // spinB_nb_samples_psd->value(), 20-1000

                     // We choose a sample if: max fold > (vertex orthogonal dir to the bone) dot (vertex normal)
                     0); // dSpinB_max_fold->value()
        } else {
            samples.choose_hrbf_samples_ad_hoc
                    (*impl->cudaCtrl._anim_mesh->_animesh,
                     bone_id,
                     -0.02f, // dSpinB_max_dist_joint->value(),
                     -0.02f, // dSpinB_max_dist_parent->value(),
                     0, // dSpinB_min_dist_samples->value(), Minimal distance between two HRBF sample
                     0); // dSpinB_max_fold->value()
        }
    }

    impl->cudaCtrl._anim_mesh->set_sampleset(samples);

    impl->cudaCtrl._anim_mesh->update_base_potential();
}

void PluginInterface::update_skeleton(const vector<Loader::CpuTransfo> &bone_positions)
{
    // Update the skeleton transforms.
    vector<Transfo> transfos(bone_positions.size());
    for(int i = 0; i < bone_positions.size(); ++i)
        transfos[i] = Transfo(bone_positions[i]);

    // If we've been given fewer transformations than there are bones, set the missing ones to identity.
    transfos.insert(transfos.end(), bone_positions.size() - bone_positions.size(), Transfo::identity());

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

int PluginInterface::expected_vertex_count() const
{
    return impl->cudaCtrl._anim_mesh->_animesh->get_nb_vertices();
}

void PluginInterface::go(vector<Loader::Vec3> &out_verts)
{
    impl->cudaCtrl._anim_mesh->set_do_smoothing(true);
    impl->cudaCtrl._anim_mesh->deform_mesh();

    impl->cudaCtrl._anim_mesh->_animesh->get_anim_vertices_aifo(out_verts);
}
