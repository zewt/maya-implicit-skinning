/*
 Implicit skinning
 Copyright (C) 2013 Rodolphe Vaillant, Loic Barthe, Florian Cannezin,
 Gael Guennebaud, Marie Paule Cani, Damien Rohmer, Brian Wyvill,
 Olivier Gourmel

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License 3 as published by
 the Free Software Foundation.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program. If not, see <http://www.gnu.org/licenses/>
 */
#include "animated_mesh_ctrl.hpp"

#include "animesh.hpp"
#include "globals.hpp"
#include "std_utils.hpp"
#include "skeleton.hpp"

// -----------------------------------------------------------------------------
namespace { __device__ void fix_debug() { } }

Animated_mesh_ctrl::Animated_mesh_ctrl(Mesh* mesh, Skeleton *skel) :
    _auto_precompute(true),
    _nb_iter(7),
    _samples(skel->nb_joints()),
    _animesh(new Animesh(mesh, skel))
{
    int n = skel->nb_joints();
}

Animated_mesh_ctrl::~Animated_mesh_ctrl()
{
    delete _animesh;
}

void Animated_mesh_ctrl::precompute_all_bones()
{
    for(int i = 0; i < _animesh->get_skel()->nb_joints(); i++)
    {
        if(_animesh->get_skel()->bone_type(i) == EBone::HRBF)
            set_bone_type( i, EBone::PRECOMPUTED);
    }
}

void Animated_mesh_ctrl::set_bone_type(int id, int bone_type)
{
    // Don't waste memory converting joints with no associated vertices.
    // XXX: but we convert to HRBF elsewhere and we can't leave bones in HRBF, even if
    // they're empty (eg. bbox will be slow)
//    if(bone_type != EBone::SSD && h_verts_id_per_bone[id].size() == 0)
//        return;

    Skeleton *skel = _animesh->get_skel();

    // Make sure that transform_hrbf/transform_precomputed_prim has been
    // called.  XXX: This probably shouldn't be needed here, or at least
    // we could only update the correct bone so we don't do n^2 updates.
    skel->update_bones_pose();

    Bone *bone = skel->get_bone( id );
    switch(bone_type){
    case EBone::PRECOMPUTED:
    {
        bone->set_enabled(true);
        bone->precompute(skel->get_skel_id());
        break;
    }
    case EBone::HRBF:
        bone->set_enabled(true);
        bone->discard_precompute();
        break;
    case EBone::SSD:
        bone->set_enabled(false);
        break;

    default: //unknown bone type !
        assert(false);
        break;

    }

    // XXX: It makes sense that we need this here, but if we don't call it we hang in a weird way.
    // Figure out why for diagnostics.
    skel->update_bones_pose();
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::set_do_smoothing(bool state)
{
    if(_animesh != 0){
        _animesh->set_smooth_mesh(state);
    }
}

void Animated_mesh_ctrl::set_smooth_factor(int i, float fact){
    _animesh->set_smooth_factor(i, fact);
}

void Animated_mesh_ctrl::enable_update_base_potential(bool state)
{
    _animesh->set_enable_update_base_potential(state);
}

void Animated_mesh_ctrl::update_base_potential()
{
    assert(_animesh != 0);
    _animesh->update_base_potential();
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::deform_mesh()
{
    _animesh->transform_vertices();
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::write_hrbf_radius( std::ofstream& file )
{
    file << "[HRBF_RADIUS]" << std::endl;
    file << "nb_bone "      << _animesh->get_skel()->nb_joints() << std::endl;

    for(int i = 0; i < _animesh->get_skel()->nb_joints(); i++ )
    {
        file << "bone_id "     << i                         << std::endl;
        const Bone *bone = _animesh->get_skel()->get_bone(i);
        file << "hrbf_radius " << bone->get_hrbf_radius() << std::endl;
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::save_ism(const char* filename)
{
    using namespace std;
    ofstream file(filename, ios_base::out|ios_base::trunc);

    if(!file.is_open()){
        cerr << "Error exporting file " << filename << endl;
        exit(1);
    }

    write_hrbf_radius( file );

    file.close();
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::read_weights(std::ifstream& file,
                                      std::vector<float4>& weights )
{
    std::string nil;
    int nb_samples = -1;
    file >> nil /*'nb_points'*/ >> nb_samples;
    weights.resize( nb_samples );

    for(int j = 0; j < nb_samples; ++j)
    {
        Vec3_cu beta;
        float alpha;
        file >> beta.x >> beta.y >> beta.z;
        file >> alpha;

        weights[j] = make_float4(beta.x, beta.y, beta.z, alpha);
    }
}

void Animated_mesh_ctrl::read_hrbf_env_weights(
        std::ifstream& file,
        std::vector<std::vector<float4> >& bone_weights)
{
    std::string nil;
    int bone_id  = -1;
    int nb_bones_file = 0;

    file >> nil/*'nb_bone'*/ >> nb_bones_file;

    for(int i = 0; i < nb_bones_file; i++ )
    {
        file >> nil/*'bone_id'*/ >> bone_id;
        read_weights(file, bone_weights[i]);
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::read_hrbf_radius(std::ifstream& file,
                                          std::vector<float>& radius_hrbf)
{
    std::string nil;
    int nb_bones = -1;

    file >> nil /*'nb_bone'*/ >> nb_bones;

    for(int i = 0; i < nb_bones; ++i)
    {
        int bone_id = -1;
        file >> nil /*'bone_id'*/     >> bone_id;
        file >> nil /*'bone_radius'*/ >> radius_hrbf[bone_id];
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::load_ism(const char* filename)
{
    using namespace std;
    ifstream file(filename);

    if(!file.is_open()) {
        cerr << "Error importing file " << filename << endl;
        return;
    }

    std::vector<float> radius_hrbf(_animesh->get_skel()->nb_joints(), -1.f      );

    std::vector<std::vector<float4> > bone_weights(_animesh->get_skel()->nb_joints());

    while( !file.eof() )
    {
        string section;
        file >> section;

        if(section == "[HRBF_ENV_WEIGHTS]") read_hrbf_env_weights(file, bone_weights);
        else if(section == "[HRBF_RADIUS]")      read_hrbf_radius( file, radius_hrbf);
        else
        {
            std::cerr << "WARNING ism import: can't read this symbol '";
            std::cerr << section << "'" << std::endl;
        }
    }
    file.close();

    for(int i = 0; i < _animesh->get_skel()->nb_joints(); i++)
    {
        if( radius_hrbf[i] > 0.f)
        {
            Bone *bone = _animesh->get_skel()->get_bone(i);
            bone->set_hrbf_radius(radius_hrbf[i]);
        }
    }

    _animesh->update_base_potential();
}

void Animated_mesh_ctrl::set_nb_iter_smooting(int nb_iter)
{
    if(_animesh != 0){
        _animesh->set_smoothing_iter(nb_iter);
        _nb_iter = nb_iter;
    }
}

void Animated_mesh_ctrl::smooth_conservative(){
    _animesh->set_smoothing_type(EAnimesh::CONSERVATIVE);
}
void Animated_mesh_ctrl::smooth_laplacian(){
    _animesh->set_smoothing_type(EAnimesh::LAPLACIAN);
}
void Animated_mesh_ctrl::smooth_tangential(){
    _animesh->set_smoothing_type(EAnimesh::TANGENTIAL);
}
void Animated_mesh_ctrl::smooth_humphrey(){
    _animesh->set_smoothing_type(EAnimesh::HUMPHREY);
}
void Animated_mesh_ctrl::set_local_smoothing(bool state){
    _animesh->set_local_smoothing(state);
}
void Animated_mesh_ctrl::set_smooth_force_a (float alpha){
    _animesh->set_smooth_force_a(alpha);
}
void Animated_mesh_ctrl::set_smooth_force_b (float beta){
    _animesh->set_smooth_force_b(beta);
}

void Animated_mesh_ctrl::set_smooth_smear(float val ){
    _animesh->set_smooth_smear(val);
}

void Animated_mesh_ctrl::set_smoothing_weights_diffusion_iter(int nb_iter){
    _animesh->set_smoothing_weights_diffusion_iter(nb_iter);
}

void Animated_mesh_ctrl::get_anim_vertices_aifo(std::vector<Point_cu>& out) const
{
    _animesh->get_anim_vertices_aifo(out);
}

void Animated_mesh_ctrl::copy_vertices(const std::vector<Vec3_cu> &vertices)
{
    _animesh->copy_vertices(vertices);
}

int Animated_mesh_ctrl::get_nb_vertices() const
{
    return _animesh->get_nb_vertices();
}

// -----------------------------------------------------------------------------

// Combine the samples in _samples, and send them to the Animesh.
void Animated_mesh_ctrl::update_bone_samples(int bone_id)
{
    SampleSet::InputSample sample_list;
    bone_id = _samples.get_all_bone_samples(*_animesh->get_skel(), bone_id, sample_list);
    _animesh->update_bone_samples(bone_id, sample_list.nodes, sample_list.n_nodes);
    if(_auto_precompute) precompute_all_bones();
}

void Animated_mesh_ctrl::set_hrbf_radius(int bone_id, float rad)
{
    Bone *bone = _animesh->get_skel()->get_bone(bone_id);
    bool was_precomputed = (bone->get_type() == EBone::PRECOMPUTED);
    bone->discard_precompute();

    bone->set_hrbf_radius(rad);

    if(was_precomputed)
        bone->precompute(skel->get_skel_id());
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::set_sampleset(const SampleSet::SampleSet &sample_set)
{
    _samples = sample_set;
    for(int bone_id = 0; bone_id < _animesh->get_skel()->nb_joints(); ++bone_id)
        update_bone_samples(bone_id);
}
