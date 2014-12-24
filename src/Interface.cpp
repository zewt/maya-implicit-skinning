// nVidia and Autodesk are both equally incompetent: they both declare types like "short3" in their
// headers with no namespacing whatsoever, which means that CUDA and Maya can't be used together
// without jumping hoops.  This file exposes an interface for the plugin that doesn't include any
// CUDA headers.  Seriously, this is a joke.

#include "Interface.h"

#include "loader_mesh.hpp"
#include "mesh.hpp"

#include "blending_lib/generator.hpp"
#include "vec3_cu.hpp"
#include "cuda_ctrl.hpp"
#include "loader.hpp"
#include "fbx_loader.hpp"
#include "conversions.hpp"
#include "cuda_ctrl.hpp"
#include "animesh.hpp"

#include <vector>
using namespace std;

void PluginInterface::init()
{
    std::vector<Blending_env::Op_t> op;
    op.push_back( Blending_env::B_D  );
    op.push_back( Blending_env::U_OH );
    op.push_back( Blending_env::C_D  );

    Cuda_ctrl::cuda_start(op);
    Cuda_ctrl::init_opengl_cuda();

    // XXX "HACK: Because blending_env initialize to elbow too ..." What?
    IBL::Ctrl_setup shape = IBL::Shape::elbow();
}

void PluginInterface::shutdown()
{
    Cuda_ctrl::cleanup();
}

extern Mesh* g_mesh;
#include "skeleton.hpp"
extern Skeleton* g_skel;





void update_hrbf_samples(int bone_id)
{
    if( g_skel->is_leaf(bone_id) )
        return;


    switch(0)
    {
    case 0:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_poisson
                (bone_id,
                 // Set a distance threshold from sample to the joints to choose them.
                 -0.02f, // dSpinB_max_dist_joint->value(),
                 -0.02f, // dSpinB_max_dist_parent->value(),
                 0, // dSpinB_min_dist_samples->value(),
                 // Minimal number of samples.  (this value is used only whe the value min dist is zero)
                 50, // spinB_nb_samples_psd->value(), 20-1000

                 // We choose a sample if: max fold > (vertex orthogonal dir to the bone) dot (vertex normal)
                 0); // dSpinB_max_fold->value() );

        break;
    }

    case 1:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_ad_hoc
                (bone_id,
                 -0.02f, // dSpinB_max_dist_joint->value(),
                 -0.02f, // dSpinB_max_dist_parent->value(),
                 0, // dSpinB_min_dist_samples->value(), Minimal distance between two HRBF sample
                 0); // dSpinB_max_fold->value() );

        break;
    }

    case 2:
    {
        Cuda_ctrl::_anim_mesh->choose_hrbf_samples_gael( bone_id );
        break;
    }
    }
}


void update_all_hrbf_samples()
{
    for(int bone_id = 0; bone_id < g_skel->nb_joints(); ++bone_id)
    {
        update_hrbf_samples(bone_id);
    }
}

// We're being given a mesh to work with.  Load it into Mesh, and hand it to the CUDA
// interface.
static Loader::Abs_skeleton original_loader_skeleton; // XXX
void PluginInterface::go(const Loader::Abs_mesh &loader_mesh, const Loader::Abs_skeleton &loader_skeleton, vector<Loader::Vec3> &out_verts)
{
    if(Cuda_ctrl::_anim_mesh == NULL)
    {
        // Abs_mesh is a simple representation that doesn't touch CUDA.  Load it into
        // Mesh.
        Mesh* ptr_mesh = new Mesh();
        ptr_mesh->load(loader_mesh);

        // Hand the Mesh to Cuda_ctrl.
        Cuda_ctrl::load_mesh( ptr_mesh );

        original_loader_skeleton = loader_skeleton;
        Cuda_ctrl::_skeleton.load(loader_skeleton);
    //    Cuda_ctrl::_skeleton.set_offset_scale( g_mesh->get_offset(), g_mesh->get_scale());

        Cuda_ctrl::load_animesh();



        update_all_hrbf_samples();


        Cuda_ctrl::_anim_mesh->update_base_potential();

//        Cuda_ctrl::_anim_mesh->save_ism("c:/foo.ism");
    }

    // Update the skeleton representation.
    {
        vector<Transfo> transfos;
        for(int i = 0; i < loader_skeleton._bones.size(); ++i)
        {
            const Loader::Abs_bone &bone = loader_skeleton._bones[i];
            Loader::CpuTransfo currentTransform = bone._frame;
            Loader::CpuTransfo bindTransform = original_loader_skeleton._bones[i]._frame;

            Loader::CpuTransfo inverted = bindTransform.full_invert();
            Loader::CpuTransfo changeToTransform = currentTransform * inverted;

            transfos.push_back(Transfo(changeToTransform));
        }
        g_skel->set_transforms(transfos);
    }

//    loader_skeleton._bones[0].
    int num_vertices = loader_mesh._vertices.size();
    vector<Vec3_cu> vertices(num_vertices);
    const Loader::Vertex *mesh_vertices = &loader_mesh._vertices[0];
    for(size_t i = 0; i < num_vertices; ++i)
    {
        vertices[i] = Vec3_cu(
            mesh_vertices[i].x,
            mesh_vertices[i].y,
            mesh_vertices[i].z);
    }
    Cuda_ctrl::_anim_mesh->_animesh->copy_vertices(vertices);

//    Cuda_ctrl::_anim_mesh->update_base_potential();

    Cuda_ctrl::_anim_mesh->set_do_smoothing(true);
    Cuda_ctrl::_anim_mesh->deform_mesh();

    Cuda_ctrl::_anim_mesh->_animesh->get_anim_vertices_aifo(out_verts);
}
