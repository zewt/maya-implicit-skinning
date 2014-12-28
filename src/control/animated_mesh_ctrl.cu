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

#include "conversions.hpp"
#include "animesh.hpp"
#include "globals.hpp"
#include "cuda_ctrl.hpp"
#include "std_utils.hpp"
#include "loader_skel.hpp"
#include "skeleton.hpp"

// -----------------------------------------------------------------------------

Animated_mesh_ctrl::Animated_mesh_ctrl(Animesh* am) :
    _auto_precompute(true),
    _factor_bones(false),
    _nb_iter(7),
    _bone_caps(am->get_skel()->nb_joints()),
    _bone_anim_caps(am->get_skel()->nb_joints()),
    _sample_list(am->get_skel()->nb_joints()),
    _sample_anim_list(am->get_skel()->nb_joints()),
    _animesh(am),
    _skel( am->get_skel() )
{
    int n = am->get_skel()->nb_joints();
    for(int i = 0; i < n; i++){
        _bone_caps[i].jcap.enable = false;
        _bone_caps[i].pcap.enable = false;
        _bone_anim_caps[i].jcap.enable = false;
        _bone_anim_caps[i].pcap.enable = false;
    }
}

// -----------------------------------------------------------------------------

Animated_mesh_ctrl::~Animated_mesh_ctrl()
{
    delete _animesh;
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::set_bone_type(int bone_id, int bone_type)
{
    if(bone_type == _skel->bone_type(bone_id)) return;

    if(bone_type == EBone::HRBF)
    {
        _animesh->set_bone_type(bone_id, bone_type);
        update_bone_samples(bone_id);
    }
    else
        _animesh->set_bone_type(bone_id, bone_type);
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

void Animated_mesh_ctrl::write_bone_types(std::ofstream& file)
{
    file << "[BONE_TYPES]" << std::endl;
    file << "nb_bone "     << _skel->nb_joints() << std::endl;

    for(int i = 0; i < _skel->nb_joints(); i++ )
    {
        file << "bone_id "   << i                         << std::endl;
        file << "bone_type " << _skel->bone_type( i ) << std::endl;
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::write_hrbf_env(std::ofstream& file)
{
    file << "[HRBF_ENV]" << std::endl;
    file << "nb_bone "   << _sample_list.size() << std::endl;

    for(unsigned i = 0; i < _sample_list.size(); i++ )
    {
        file << "bone_id " << i << std::endl;

        write_samples(file, _sample_list[i].nodes, _sample_list[i].n_nodes);
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::write_hrbf_caps_env(std::ofstream& file, bool jcap)
{
    if( jcap ) file << "[HRBF_JCAPS_ENV]" << std::endl;
    else       file << "[HRBF_PCAPS_ENV]" << std::endl;

    file << "nb_bone " << _bone_caps.size() << std::endl;

    for(unsigned i = 0; i < _bone_caps.size(); i++ )
    {
        const Cap&  cap_list = jcap ? _bone_caps[i].jcap : _bone_caps[i].pcap;
        const float rad      = _animesh->get_junction_radius(i);

        file << "bone_id "       << i               << std::endl;
        file << "is_cap_enable " << cap_list.enable << std::endl;
        file << "cap_radius "    << rad             << std::endl;
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::write_samples(std::ofstream& file,
                                       const std::vector<Vec3_cu>& nodes,
                                       const std::vector<Vec3_cu>& n_nodes )
{
    assert(nodes.size() == n_nodes.size());

    file << "nb_points " << nodes.size() << std::endl;

    for(unsigned j = 0; j < nodes.size(); ++j)
    {
        const Vec3_cu pt = nodes  [j];
        const Vec3_cu n  = n_nodes[j];
        file << pt.x << " " << pt.y << " " << pt.z << std::endl;
        file << n.x  << " " << n.y  << " " << n.z  << std::endl;
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::write_hrbf_radius( std::ofstream& file )
{
    file << "[HRBF_RADIUS]" << std::endl;
    file << "nb_bone "      << _skel->nb_joints() << std::endl;

    for(int i = 0; i < _skel->nb_joints(); i++ )
    {
        file << "bone_id "     << i                         << std::endl;
        file << "hrbf_radius " << _skel->get_hrbf_radius(i) << std::endl;
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

    write_hrbf_env( file );
    write_hrbf_caps_env( file, true  /*write joint caps*/);
    write_hrbf_caps_env( file, false /*write parent caps*/);
    write_bone_types( file );
    write_hrbf_radius( file );

    file.close();
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::read_bone_types(std::ifstream& file,
                                         std::vector<int>& bones_type)
{
    std::string nil;
    int nb_bones = -1;

    file >> nil /*'nb_bone'*/ >> nb_bones;

    for(int i = 0; i < nb_bones; i++)
    {
        int bone_id = -1;
        file >> nil /*'bone_id'*/    >> bone_id;
        file >> nil /*'bone_type '*/ >> bones_type[bone_id];
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::read_samples(std::ifstream& file,
                                      std::vector<Vec3_cu>& nodes,
                                      std::vector<Vec3_cu>& n_nodes )
{
    assert(nodes.size() == n_nodes.size());

    std::string nil;
    int nb_samples = -1;
    file >> nil /*'nb_points'*/ >> nb_samples;
    nodes.  resize( nb_samples );
    n_nodes.resize( nb_samples );

    for(int j = 0; j < nb_samples; ++j)
    {
        Vec3_cu pt, n;
        file >> pt.x >> pt.y >> pt.z;
        file >> n.x  >> n.y  >> n.z;

//        Transfo tr = Transfo::rotate(Vec3_cu::unit_x(), M_PI) * Transfo::rotate(Vec3_cu::unit_z(), M_PI);// DEBUG---
        nodes  [j] = /*tr */ pt;
        n_nodes[j] = /*tr */ n;
    }
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

        Cap& cap_list = jcap ? _bone_caps[bone_id].jcap : _bone_caps[bone_id].pcap;
        file >> nil /*'is_cap_enable'*/ >> cap_list.enable;
        file >> nil /*'cap_radius'*/    >> rad;
        _animesh->set_junction_radius(bone_id, rad);
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::read_hrbf_env(std::ifstream& file)
{
    std::string nil;
    int bone_id  = -1;
    int nb_bones_file = 0;

    file >> nil/*'nb_bone'*/ >> nb_bones_file;

    for(int i = 0; i<nb_bones_file; i++ )
    {
        file >> nil/*'bone_id'*/ >> bone_id;
        read_samples(file, _sample_list[bone_id].nodes, _sample_list[bone_id].n_nodes);
        Std_utils::copy( _sample_anim_list[bone_id].  nodes, _sample_list[bone_id].  nodes);
        Std_utils::copy( _sample_anim_list[bone_id].n_nodes, _sample_list[bone_id].n_nodes);
    }
}

// -----------------------------------------------------------------------------

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

    std::vector<int>   bones_type (_skel->nb_joints(), EBone::SSD);
    std::vector<float> radius_hrbf(_skel->nb_joints(), -1.f      );

    std::vector<std::vector<float4> > bone_weights(_skel->nb_joints());

    while( !file.eof() )
    {
        string section;
        file >> section;

        if(section == "[HRBF_ENV]")              read_hrbf_env( file );
        else if(section == "[HRBF_ENV_WEIGHTS]") read_hrbf_env_weights(file, bone_weights);
        else if(section == "[HRBF_JCAPS_ENV]")   read_hrbf_caps_env( file, true  );
        else if(section == "[HRBF_PCAPS_ENV]")   read_hrbf_caps_env( file, false );
        else if(section == "[BONE_TYPES]")       read_bone_types( file, bones_type );
        else if(section == "[HRBF_RADIUS]")      read_hrbf_radius( file, radius_hrbf);
        else
        {
            std::cerr << "WARNING ism import: can't read this symbol '";
            std::cerr << section << "'" << std::endl;
        }
    }
    file.close();

    for(int i = 0; i < _skel->nb_joints(); i++)
    {
        const EBone::Bone_t t = (EBone::Bone_t)bones_type[i];

        if( radius_hrbf[i] > 0.f)
            _skel->set_bone_hrbf_radius(i, radius_hrbf[i]);

        update_caps(i, true, true);

        if(t == EBone::HRBF || t == EBone::PRECOMPUTED)
        {
            // Check if weights are here and don't update if not necessary
            //...

            // This call will compute hrbf weights and convert to precomputed
            // if _auto_precompute == true
            update_bone_samples(i);

            if( !_auto_precompute && t == EBone::PRECOMPUTED)
                _animesh->set_bone_type( i, EBone::PRECOMPUTED);
        }
        else
            set_bone_type(i, t);
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

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::update_bone_samples(int bone_id)
{
    std::vector<Vec3_cu> nodes, n_nodes;

    if( _factor_bones && bone_id != _skel->root() )
    {
        int parent = _skel->parent( bone_id );
        const std::vector<int>& sons = _skel->get_sons( parent );
        assert(sons.size() > 0);
        int son0 = sons[0];

        //if(son0 != bone_id) // In factor bone mode samples are always in the first son
            //return;
        // <- yea but caps are not stored in the first son ...

        int nb_nodes = _sample_list[son0].nodes.size();
        nodes.  resize( nb_nodes );
        n_nodes.resize( nb_nodes );
        resize_samples_anim(son0, nb_nodes);
        for(int i = 0; i < nb_nodes; i++) {
            nodes  [i] = _sample_list[son0].nodes  [i];
            n_nodes[i] = _sample_list[son0].n_nodes[i];
        }

        for( unsigned i = 0; i < sons.size(); i++) {
            int bid = sons[i];
            if(_bone_caps[bid].jcap.enable && !_skel->is_leaf(bid))
            {
                for(unsigned j = 0; j < _bone_caps[bid].jcap.nodes.size(); j++){
                    nodes.  push_back( _bone_caps[bid].jcap.nodes  [j] );
                    n_nodes.push_back( _bone_caps[bid].jcap.n_nodes[j] );
                }
            }
        }

        if(_bone_caps[son0].pcap.enable) {
            for(unsigned i = 0; i < _bone_caps[son0].pcap.nodes.size(); i++){
                nodes.  push_back( _bone_caps[son0].pcap.nodes  [i] );
                n_nodes.push_back( _bone_caps[son0].pcap.n_nodes[i] );
            }
        }

        _animesh->update_bone_samples(son0, nodes, n_nodes);
    }
    else
    {
        int nb_nodes = _sample_list[bone_id].nodes.size();
        nodes.  resize( nb_nodes );
        n_nodes.resize( nb_nodes );
        resize_samples_anim(bone_id, nb_nodes);

        // Concat nodes and normals
        for(int i = 0; i < nb_nodes; i++)
        {
            nodes  [i] = _sample_list[bone_id].nodes  [i];
            n_nodes[i] = _sample_list[bone_id].n_nodes[i];
        }
        // concat joint caps
        if(_bone_caps[bone_id].jcap.enable && !_skel->is_leaf(bone_id))
        {
            for(unsigned j = 0; j < _bone_caps[bone_id].jcap.nodes.size(); j++) {
                nodes.  push_back( _bone_caps[bone_id].jcap.nodes  [j] );
                n_nodes.push_back( _bone_caps[bone_id].jcap.n_nodes[j] );
            }
        }
        // concat parent caps
        if(_bone_caps[bone_id].pcap.enable)
        {
            for(unsigned i = 0; i < _bone_caps[bone_id].pcap.nodes.size(); i++){
                nodes.  push_back( _bone_caps[bone_id].pcap.nodes  [i] );
                n_nodes.push_back( _bone_caps[bone_id].pcap.n_nodes[i] );
            }
        }
        _animesh->update_bone_samples(bone_id, nodes, n_nodes);
    }

    if(_auto_precompute) precompute_all_bones();
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::update_caps(int bone_id, bool jcap, bool pcap)
{
    std::vector<Vec3_cu>& jnodes   = _bone_caps[bone_id].jcap.nodes;
    std::vector<Vec3_cu>& jn_nodes = _bone_caps[bone_id].jcap.n_nodes;

    std::vector<Vec3_cu>& pnodes   = _bone_caps[bone_id].pcap.nodes;
    std::vector<Vec3_cu>& pn_nodes = _bone_caps[bone_id].pcap.n_nodes;

    if(jcap && !_skel->is_leaf(bone_id))
    {
        jnodes.  clear();
        jn_nodes.clear();
        _animesh->compute_jcaps(bone_id, jnodes, jn_nodes);
    }

    if(pcap)
    {
        int parent = _skel->parent( bone_id );
        if(_factor_bones && parent != -1)
        {
            const std::vector<int>& sons = _skel->get_sons( parent );
            assert(sons.size() > 0);
            _bone_caps[ sons[0] ].pcap.nodes.  clear();
            _bone_caps[ sons[0] ].pcap.n_nodes.clear();
            _animesh->compute_pcaps(sons[0],
                                    (sons.size() > 1),
                                    _bone_caps[ sons[0] ].pcap.nodes,
                                    _bone_caps[ sons[0] ].pcap.n_nodes);
        }
        else
        {
            pnodes.  clear();
            pn_nodes.clear();
            _animesh->compute_pcaps(bone_id, false, pnodes, pn_nodes);
        }
    }

    _bone_anim_caps[bone_id].jcap.enable = _bone_caps[bone_id].jcap.enable;
    _bone_anim_caps[bone_id].jcap.nodes.  resize( jnodes.  size() );
    _bone_anim_caps[bone_id].jcap.n_nodes.resize( jn_nodes.size() );

    _bone_anim_caps[bone_id].pcap.enable = _bone_caps[bone_id].pcap.enable;
    _bone_anim_caps[bone_id].pcap.nodes.  resize( pnodes.  size() );
    _bone_anim_caps[bone_id].pcap.n_nodes.resize( pn_nodes.size() );
}


void Animated_mesh_ctrl::update_hrbf_samples(int bone_id, int mode)
{
    if( _skel->is_leaf(bone_id) )
        return;

    switch(mode)
    {
    case 0:
    {
        choose_hrbf_samples_poisson
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
        choose_hrbf_samples_ad_hoc
                (bone_id,
                 -0.02f, // dSpinB_max_dist_joint->value(),
                 -0.02f, // dSpinB_max_dist_parent->value(),
                 0, // dSpinB_min_dist_samples->value(), Minimal distance between two HRBF sample
                 0); // dSpinB_max_fold->value() );

        break;
    }
    }
}

void Animated_mesh_ctrl::choose_hrbf_samples_ad_hoc(int bone_id,
                                                    float jmax,
                                                    float pmax,
                                                    float minDist,
                                                    float fold)
{
    Animesh::Adhoc_sampling heur(_animesh);
    heur._bone_id = bone_id;
    heur._jmax = jmax;
    heur._pmax = pmax;
    heur._mind = minDist;
    heur._fold = fold;
    heur._factor_siblings = _factor_bones;

    update_caps(bone_id, true, true); // FIXME: dunno why it's here

    _sample_list[bone_id].nodes.  clear();
    _sample_list[bone_id].n_nodes.clear();

    heur.sample(_sample_list[bone_id].nodes,
                _sample_list[bone_id].n_nodes);

    update_bone_samples(bone_id);
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::choose_hrbf_samples_poisson(int bone_id,
                                                     float jmax,
                                                     float pmax,
                                                     float minDist,
                                                     int nb_samples,
                                                     float fold)
{
    Animesh::Poisson_disk_sampling heur(_animesh);
    heur._bone_id = bone_id;
    heur._jmax = jmax;
    heur._pmax = pmax;
    heur._mind = minDist;
    heur._nb_samples = nb_samples;
    heur._fold = fold;
    heur._factor_siblings = _factor_bones;

    update_caps(bone_id, true, true); // FIXME: dunno why it's here

    _sample_list[bone_id].nodes.  clear();
    _sample_list[bone_id].n_nodes.clear();

    heur.sample(_sample_list[bone_id].nodes,
                _sample_list[bone_id].n_nodes);

    update_bone_samples(bone_id);
}

void Animated_mesh_ctrl::set_hrbf_radius(int bone_id, float rad)
{
    if(_skel->bone_type(bone_id) == EBone::HRBF)
        _skel->set_bone_hrbf_radius( bone_id, rad);
    else if( _skel->bone_type(bone_id) == EBone::PRECOMPUTED )
    {
        bool tmp = _auto_precompute;
        // disable _auto_precompute. otherwise set_bone_type() will convert
        // back to precomputed the bone without letting us the chance to change
        // the radius first
        _auto_precompute = false;
        this->set_bone_type(bone_id, EBone::HRBF);
        _skel->set_bone_hrbf_radius( bone_id, rad);
        this->set_bone_type(bone_id, EBone::PRECOMPUTED);
        _auto_precompute = tmp;
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::delete_sample(int bone_id, int idx)
{
    std::vector<Vec3_cu>::iterator it = _sample_list[bone_id].nodes.begin();
    _sample_list[bone_id].nodes  .erase( it+idx );
    it = _sample_list[bone_id].n_nodes.begin();
    _sample_list[bone_id].n_nodes.erase( it+idx );
    update_bone_samples(bone_id);
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::empty_samples(int bone_id)
{
    _sample_list[bone_id].nodes.  clear();
    _sample_list[bone_id].n_nodes.clear();

    update_bone_samples(bone_id);
}

// -----------------------------------------------------------------------------

int Animated_mesh_ctrl::add_sample(int bone_id,
                                   const Vec3_cu& p,
                                   const Vec3_cu& n)
{
    _sample_list[bone_id].nodes.  push_back(p);
    _sample_list[bone_id].n_nodes.push_back(n);

    update_bone_samples(bone_id);

    return _sample_list[bone_id].nodes.size()-1;
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::add_samples(int bone_id,
                                     const std::vector<Vec3_cu>& p,
                                     const std::vector<Vec3_cu>& n)
{
    assert(p.size() == n.size());

    _sample_list[bone_id].nodes.  reserve( _sample_list[bone_id].  nodes.size() + p.size() );
    _sample_list[bone_id].n_nodes.reserve( _sample_list[bone_id].n_nodes.size() + n.size() );

    for(unsigned i = 0; i < p.size(); i++)
    {
        _sample_list[bone_id].nodes.  push_back(p[i]);
        _sample_list[bone_id].n_nodes.push_back(n[i]);
    }

    update_bone_samples(bone_id);
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::incr_junction_rad(int bone_id, float incr)
{
    float r = _animesh->get_junction_radius(bone_id);
    _animesh->set_junction_radius(bone_id, r+incr);

    // Recompute caps position
    update_caps(bone_id,
                false,
                _bone_caps[bone_id].pcap.enable);

    update_bone_samples(bone_id);

    int pt = _skel->parent( bone_id );
    if( pt > -1)
    {
        update_caps(pt,
                    _bone_caps[pt].jcap.enable,
                    false);

        update_bone_samples(pt);

        /*
        const std::vector<int>& c = _skel->get_sons(pt);
        for(unsigned i = 0; i < c.size(); i++)
        {
            const int cbone = c[i];
            // Recompute caps position
            update_caps(cbone,
                        _bone_caps[cbone].jcap.enable,
                        false);

            update_bone_samples(cbone);
        }
        */
    }

}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::set_jcap(int bone_id, bool state)
{
    _bone_caps     [bone_id].jcap.enable = state;
    _bone_anim_caps[bone_id].jcap.enable = state;

    update_caps(bone_id,
                state,
                _bone_caps[bone_id].pcap.enable);

    update_bone_samples(bone_id);
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::set_pcap(int bone_id, bool state)
{
    _bone_caps     [bone_id].pcap.enable = state;
    _bone_anim_caps[bone_id].pcap.enable = state;

    update_caps(bone_id,
                _bone_caps[bone_id].jcap.enable,
                state);

    update_bone_samples(bone_id);
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::transform_samples(const std::vector<int>& bone_ids)
{
    int acc = 0;
    int nb_bones = bone_ids.size() == 0 ? _sample_list.size() : bone_ids.size();
    for(int i = 0; i < nb_bones; i++)
    {
        // Transform samples
        const int bone_id = bone_ids.size() == 0 ? i : bone_ids[i];
        const Transfo& tr = _skel->get_transfo( bone_id );

//        assert(_sample_anim_list[bone_id].nodes.  size() == _sample_list[bone_id].nodes.  size() );
//        assert(_sample_anim_list[bone_id].n_nodes.size() == _sample_list[bone_id].n_nodes.size() );
        if(_sample_anim_list[bone_id].nodes.size() != _sample_list[bone_id].nodes.size())
            resize_samples_anim(bone_id, _sample_list[bone_id].nodes.size());

        for(unsigned j = 0; j < _sample_list[bone_id].nodes.size(); j++)
        {
            Point_cu  p = Convs::to_point(_sample_list[bone_id].nodes[j]);
            Vec3_cu n = _sample_list[bone_id].n_nodes[j];

            Vec3_cu pos = Convs::to_vector(tr * p);
            Vec3_cu nor = tr * n;
            _sample_anim_list[bone_id].nodes  [j] = pos;
            _sample_anim_list[bone_id].n_nodes[j] = nor;

            acc++;
        }

        transform_caps(bone_id, tr);
    }
}

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::transform_caps(int bone_id, const Transfo& tr)
{
    // Transform jcaps
    if(_bone_caps[bone_id].jcap.enable)
        for(unsigned j = 0; j < _bone_caps[bone_id].jcap.nodes.size(); j++)
        {
            Point_cu p = Convs::to_point(_bone_caps[bone_id].jcap.nodes[j]);
            Vec3_cu  n = _bone_caps[bone_id].jcap.n_nodes[j];
            _bone_anim_caps[bone_id].jcap.nodes  [j] = Convs::to_vector(tr * p);
            _bone_anim_caps[bone_id].jcap.n_nodes[j] = tr * n;
        }

    // Transform pcaps
    if(_bone_caps[bone_id].pcap.enable)
        for(unsigned j = 0; j < _bone_caps[bone_id].pcap.nodes.size(); j++)
        {
            Point_cu p = Convs::to_point(_bone_caps[bone_id].pcap.nodes[j]);
            Vec3_cu  n = _bone_caps[bone_id].pcap.n_nodes[j];
            _bone_anim_caps[bone_id].pcap.nodes  [j] = Convs::to_vector(tr * p);
            _bone_anim_caps[bone_id].pcap.n_nodes[j] = tr * n;
        }
}

void Animated_mesh_ctrl::resize_samples_anim(int bone_id, int size)
{
    _sample_anim_list[bone_id].nodes.  resize( size );
    _sample_anim_list[bone_id].n_nodes.resize( size );
}

int Animated_mesh_ctrl::compute_nb_samples()
{
    int acc = 0;
    for(unsigned i = 0; i < _sample_list.size(); i++)
        acc += _sample_list[i].nodes.size();

    return acc;
}

void Animated_mesh_ctrl::precompute_all_bones()
{
    for(int i = 0; i < _skel->nb_joints(); i++)
    {
        int type = _skel->bone_type( i );
        if(type != EBone::PRECOMPUTED && type != EBone::SSD)
            _animesh->set_bone_type( i, EBone::PRECOMPUTED);
    }
}
