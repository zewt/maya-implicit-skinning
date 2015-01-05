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
            _animesh->set_bone_type( i, EBone::PRECOMPUTED);
    }
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

void Animated_mesh_ctrl::update_clusters(int nb_voxels)
{
    _animesh->clusterize(nb_voxels);
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::deform_mesh()
{
    _animesh->transform_vertices();
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::write_hrbf_caps_env(std::ofstream& file, bool jcap)
{
    if( jcap ) file << "[HRBF_JCAPS_ENV]" << std::endl;
    else       file << "[HRBF_PCAPS_ENV]" << std::endl;

    file << "nb_bone " << _samples._samples.size() << std::endl;

    for(unsigned i = 0; i < _samples._samples.size(); i++ )
    {
        const SampleSet::Cap&  cap_list = jcap ? _samples._samples[i].jcap : _samples._samples[i].pcap;
        const float rad      = _samples._samples[i]._junction_radius;

        file << "bone_id "       << i               << std::endl;
        file << "is_cap_enable " << cap_list.enable << std::endl;
        file << "cap_radius "    << rad             << std::endl;
    }
}

void Animated_mesh_ctrl::write_hrbf_radius( std::ofstream& file )
{
    file << "[HRBF_RADIUS]" << std::endl;
    file << "nb_bone "      << _animesh->get_skel()->nb_joints() << std::endl;

    for(int i = 0; i < _animesh->get_skel()->nb_joints(); i++ )
    {
        file << "bone_id "     << i                         << std::endl;
        file << "hrbf_radius " << _animesh->get_skel()->get_hrbf_radius(i) << std::endl;
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

    write_hrbf_caps_env( file, true  /*write joint caps*/);
    write_hrbf_caps_env( file, false /*write parent caps*/);
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

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::read_hrbf_caps_env(std::ifstream& file, bool jcap)
{
    std::string nil;
    int nb_bones = -1;

    file >> nil /*'nb_bone'*/ >> nb_bones;

    for(int i = 0; i < nb_bones; i++ )
    {
        float rad;
        int bone_id = -1;
        file >> nil /*'bone_id'*/ >> bone_id;

        SampleSet::Cap& cap_list = jcap ? _samples._samples[bone_id].jcap : _samples._samples[bone_id].pcap;
        file >> nil /*'is_cap_enable'*/ >> cap_list.enable;
        file >> nil /*'cap_radius'*/    >> rad;
        _samples._samples[bone_id]._junction_radius = rad;
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
        else if(section == "[HRBF_JCAPS_ENV]")   read_hrbf_caps_env( file, true  );
        else if(section == "[HRBF_PCAPS_ENV]")   read_hrbf_caps_env( file, false );
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
            _animesh->get_skel()->set_bone_hrbf_radius(i, radius_hrbf[i]);

        _samples.update_caps(*_animesh->get_skel(), i, true, true);
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

int Animated_mesh_ctrl::get_nearest_bone(int vert_idx){
    return _animesh->get_nearest_bone(vert_idx);
}

void Animated_mesh_ctrl::get_anim_vertices_aifo(std::vector<Point_cu>& out) const
{
    _animesh->get_anim_vertices_aifo(out);
}

void Animated_mesh_ctrl::copy_vertices(const std::vector<Vec3_cu> &vertices)
{
    _animesh->copy_vertices(vertices);
}

void Animated_mesh_ctrl::get_default_junction_radius(std::vector<float> &radius_per_joint) const
{
    _animesh->get_default_junction_radius(radius_per_joint);
}

int Animated_mesh_ctrl::get_nb_vertices() const
{
    return _animesh->get_nb_vertices();
}

Skeleton *Animated_mesh_ctrl::get_skel() { return _animesh->get_skel(); }
const Skeleton *Animated_mesh_ctrl::get_skel() const { return _animesh->get_skel(); }

// -----------------------------------------------------------------------------

// Combine the samples in _samples, and send them to the Animesh.
void Animated_mesh_ctrl::update_bone_samples(int bone_id)
{
    SampleSet::HSample_list sample_list;
    bone_id = _samples.get_all_bone_samples(*_animesh->get_skel(), bone_id, sample_list);
    _animesh->update_bone_samples(bone_id, sample_list.nodes, sample_list.n_nodes);
    if(_auto_precompute) precompute_all_bones();
}

void Animated_mesh_ctrl::set_hrbf_radius(int bone_id, float rad)
{
    if(_animesh->get_skel()->bone_type(bone_id) == EBone::HRBF)
        _animesh->get_skel()->set_bone_hrbf_radius( bone_id, rad);
    else if( _animesh->get_skel()->bone_type(bone_id) == EBone::PRECOMPUTED )
    {
        bool tmp = _auto_precompute;
        // disable _auto_precompute. otherwise set_bone_type() will convert
        // back to precomputed the bone without letting us the chance to change
        // the radius first
        // XXX: doing this directly with _animesh->set_bone_type, is this still needed?
        _auto_precompute = false;
        _animesh->set_bone_type(bone_id, EBone::HRBF);
        _animesh->get_skel()->set_bone_hrbf_radius( bone_id, rad);
        _animesh->set_bone_type(bone_id, EBone::PRECOMPUTED);
        _auto_precompute = tmp;
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::set_sampleset(const SampleSet::SampleSet &sample_set)
{
    _samples = sample_set;
    for(int bone_id = 0; bone_id < _animesh->get_skel()->nb_joints(); ++bone_id)
    {
        _samples.update_caps(*_animesh->get_skel(), bone_id, true, true); // FIXME: dunno why it's here
        update_bone_samples(bone_id);
    }
}

void Animated_mesh_ctrl::incr_junction_rad(int bone_id, float incr)
{
    _samples._samples[bone_id]._junction_radius += incr;

    // Recompute caps position
    _samples.update_caps(*_animesh->get_skel(), bone_id,
                false,
                _samples._samples[bone_id].pcap.enable);

    int pt = _animesh->get_skel()->parent( bone_id );
    if( pt > -1)
    {
        _samples.update_caps(*_animesh->get_skel(), pt,
                    _samples._samples[pt].jcap.enable,
                    false);

        /*
        const std::vector<int>& c = _animesh->get_skel()->get_sons(pt);
        for(unsigned i = 0; i < c.size(); i++)
        {
            const int cbone = c[i];
            // Recompute caps position
            update_caps(cbone,
                        _samples._samples[cbone].jcap.enable,
                        false);
        }
        */
    }

    update_bone_samples(bone_id);
}
