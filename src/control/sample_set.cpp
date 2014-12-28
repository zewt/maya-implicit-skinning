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

#include "sample_set.hpp"
#include "animesh.hpp"
#include "conversions.hpp"
#include "skeleton.hpp"

#include <sstream>

void SampleSet::read_samples(std::ifstream& file,
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

void SampleSet::read_hrbf_env(std::ifstream& file)
{
    std::string nil;
    int bone_id  = -1;
    int nb_bones_file = 0;

    file >> nil/*'nb_bone'*/ >> nb_bones_file;

    for(int i = 0; i<nb_bones_file; i++ )
    {
        file >> nil/*'bone_id'*/ >> bone_id;
        read_samples(file, _sample_list[bone_id].nodes, _sample_list[bone_id].n_nodes);
    }
}

void SampleSet::choose_hrbf_samples_ad_hoc(const Animesh &animesh,
                                                    int bone_id,
                                                    float jmax,
                                                    float pmax,
                                                    float minDist,
                                                    float fold)
{
    if(animesh.get_skel()->is_leaf(bone_id))
        return;

    Animesh::Adhoc_sampling heur(animesh);
    heur._bone_id = bone_id;
    heur._jmax = jmax;
    heur._pmax = pmax;
    heur._mind = minDist;
    heur._fold = fold;
    heur._factor_siblings = _factor_bones;

    _sample_list[bone_id].nodes.  clear();
    _sample_list[bone_id].n_nodes.clear();

    heur.sample(_sample_list[bone_id].nodes,
                _sample_list[bone_id].n_nodes);
}

void SampleSet::choose_hrbf_samples_poisson(const Animesh &animesh,
                                                     int bone_id,
                                                     float jmax,
                                                     float pmax,
                                                     float minDist,
                                                     int nb_samples,
                                                     float fold)
{
    if(animesh.get_skel()->is_leaf(bone_id))
        return;

    Animesh::Poisson_disk_sampling heur(animesh);
    heur._bone_id = bone_id;
    heur._jmax = jmax;
    heur._pmax = pmax;
    heur._mind = minDist;
    heur._nb_samples = nb_samples;
    heur._fold = fold;
    heur._factor_siblings = _factor_bones;

    _sample_list[bone_id].nodes.  clear();
    _sample_list[bone_id].n_nodes.clear();

    heur.sample(_sample_list[bone_id].nodes,
                _sample_list[bone_id].n_nodes);
}

void SampleSet::compute_jcaps(const Skeleton &skel, int bone_id,
                                 std::vector<Vec3_cu>& out_verts,
                                 std::vector<Vec3_cu>& out_normals) const
{
    const Bone* b = skel.get_bone(bone_id);

    float jrad = 0.f;// joint radius mean radius
    int nb_sons = skel.get_sons(bone_id).size();
    for (int i = 0; i < nb_sons; ++i)
        jrad += _junction_radius[ skel.get_sons(bone_id)[i] ];

    jrad /= nb_sons > 0 ? (float)nb_sons : 1.f;

    Point_cu p = b->end() + b->dir().normalized() * jrad;
    Vec3_cu  n = b->dir().normalized();
    out_verts.  push_back( p );
    out_normals.push_back( n );

    //add_circle(jrad, b->get_org(), n, out_verts, out_normals);
}

void SampleSet::compute_pcaps(const Skeleton &skel, int bone_id,
                                 bool use_parent_dir,
                                 std::vector<Vec3_cu>& out_verts,
                                 std::vector<Vec3_cu>& out_normals) const
{
    const Bone* b = skel.get_bone(bone_id);
    int parent = skel.parent(bone_id);
    float prad = _junction_radius[bone_id]; // parent joint radius
    Vec3_cu p;
    Vec3_cu n;

    if( use_parent_dir)
    {
        const Bone* pb = skel.get_bone( parent );
        p =  pb->end() - pb->dir().normalized() * prad;
        n = -pb->dir().normalized();
    }
    else
    {
        p =  b->org() - b->dir().normalized() * prad;
        n = -b->dir().normalized();
    }
    out_verts.  push_back(p);
    out_normals.push_back(n);
    //add_circle(prad, b->get_org() + b->get_dir(), n, out_verts, out_normals);
}


void SampleSet::update_caps(const Skeleton &skel, int bone_id, bool jcap, bool pcap)
{
    std::vector<Vec3_cu>& jnodes   = _bone_caps[bone_id].jcap.nodes;
    std::vector<Vec3_cu>& jn_nodes = _bone_caps[bone_id].jcap.n_nodes;

    std::vector<Vec3_cu>& pnodes   = _bone_caps[bone_id].pcap.nodes;
    std::vector<Vec3_cu>& pn_nodes = _bone_caps[bone_id].pcap.n_nodes;

    if(jcap && !skel.is_leaf(bone_id))
    {
        jnodes.  clear();
        jn_nodes.clear();
        compute_jcaps(skel, bone_id, jnodes, jn_nodes);
    }

    if(pcap)
    {
        int parent = skel.parent( bone_id );
        if(_factor_bones && parent != -1)
        {
            const std::vector<int>& sons = skel.get_sons( parent );
            assert(sons.size() > 0);
            _bone_caps[ sons[0] ].pcap.nodes.  clear();
            _bone_caps[ sons[0] ].pcap.n_nodes.clear();
            compute_pcaps(skel, sons[0],
                                    (sons.size() > 1),
                                    _bone_caps[ sons[0] ].pcap.nodes,
                                    _bone_caps[ sons[0] ].pcap.n_nodes);
        }
        else
        {
            pnodes.  clear();
            pn_nodes.clear();
            compute_pcaps(skel, bone_id, false, pnodes, pn_nodes);
        }
    }
}

void SampleSet::set_jcap(int bone_id, bool state)
{
    _bone_caps[bone_id].jcap.enable = state;
}

void SampleSet::set_pcap(int bone_id, bool state)
{
    _bone_caps[bone_id].pcap.enable = state;
}

void SampleSet::delete_sample(int bone_id, int idx)
{
    std::vector<Vec3_cu>::iterator it = _sample_list[bone_id].nodes.begin();
    _sample_list[bone_id].nodes  .erase( it+idx );
    it = _sample_list[bone_id].n_nodes.begin();
    _sample_list[bone_id].n_nodes.erase( it+idx );
}

// -----------------------------------------------------------------------------

void SampleSet::clear(int bone_id)
{
    _sample_list[bone_id].nodes.  clear();
    _sample_list[bone_id].n_nodes.clear();
}

// -----------------------------------------------------------------------------

int SampleSet::add_sample(int bone_id,
                                   const Vec3_cu& p,
                                   const Vec3_cu& n)
{
    _sample_list[bone_id].nodes.  push_back(p);
    _sample_list[bone_id].n_nodes.push_back(n);

    return _sample_list[bone_id].nodes.size()-1;
}

// -----------------------------------------------------------------------------

void SampleSet::add_samples(int bone_id,
                                     const std::vector<Vec3_cu>& p,
                                     const std::vector<Vec3_cu>& n)
{
    assert(p.size() == n.size());

    _sample_list[bone_id].nodes.insert(_sample_list[bone_id].nodes.end(), p.begin(), p.end());
    _sample_list[bone_id].n_nodes.insert(_sample_list[bone_id].n_nodes.end(), n.begin(), n.end());
}

void SampleSet::write_hrbf_env(std::ofstream& file) const
{
    file << "[HRBF_ENV]" << std::endl;
    file << "nb_bone "   << _sample_list.size() << std::endl;

    for(unsigned i = 0; i < _sample_list.size(); i++ )
    {
        file << "bone_id " << i << std::endl;

        write_samples(file, _sample_list[i].nodes, _sample_list[i].n_nodes);
    }
}

void SampleSet::write_samples(std::ofstream& file,
                                       const std::vector<Vec3_cu>& nodes,
                                       const std::vector<Vec3_cu>& n_nodes ) const
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

void SampleSet::transform_samples(const std::vector<Transfo> &transfos, const std::vector<int>& bone_ids)
{
    int acc = 0;
    int nb_bones = bone_ids.size() == 0 ? _sample_list.size() : bone_ids.size();
    for(int i = 0; i < nb_bones; i++)
    {
        // Transform samples
        const int bone_id = bone_ids.size() == 0 ? i : bone_ids[i];
        const Transfo& tr = transfos[bone_id];

        for(unsigned j = 0; j < _sample_list[bone_id].nodes.size(); j++)
        {
            Point_cu  p = Convs::to_point(_sample_list[bone_id].nodes[j]);
            Vec3_cu n = _sample_list[bone_id].n_nodes[j];

            Vec3_cu pos = Convs::to_vector(tr * p);
            Vec3_cu nor = tr * n;
            _sample_list[bone_id].nodes  [j] = pos;
            _sample_list[bone_id].n_nodes[j] = nor;

            acc++;
        }

        transform_caps(bone_id, tr);
    }
}

void SampleSet::transform_caps(int bone_id, const Transfo& tr)
{
    // Transform jcaps
    if(_bone_caps[bone_id].jcap.enable)
        for(unsigned j = 0; j < _bone_caps[bone_id].jcap.nodes.size(); j++)
        {
            Point_cu p = Convs::to_point(_bone_caps[bone_id].jcap.nodes[j]);
            Vec3_cu  n = _bone_caps[bone_id].jcap.n_nodes[j];
            _bone_caps[bone_id].jcap.nodes  [j] = Convs::to_vector(tr * p);
            _bone_caps[bone_id].jcap.n_nodes[j] = tr * n;
        }

    // Transform pcaps
    if(_bone_caps[bone_id].pcap.enable)
        for(unsigned j = 0; j < _bone_caps[bone_id].pcap.nodes.size(); j++)
        {
            Point_cu p = Convs::to_point(_bone_caps[bone_id].pcap.nodes[j]);
            Vec3_cu  n = _bone_caps[bone_id].pcap.n_nodes[j];
            _bone_caps[bone_id].pcap.nodes  [j] = Convs::to_vector(tr * p);
            _bone_caps[bone_id].pcap.n_nodes[j] = tr * n;
        }
}

int SampleSet::compute_nb_samples() const
{
    int acc = 0;
    for(unsigned i = 0; i < _sample_list.size(); i++)
        acc += _sample_list[i].nodes.size();

    return acc;
}

int SampleSet::get_all_bone_samples(const Skeleton &skel, int bone_id, std::vector<Vec3_cu> &nodes, std::vector<Vec3_cu> &n_nodes) const
{
    if( _factor_bones && bone_id != skel.root() )
    {
        int parent = skel.parent( bone_id );
        const std::vector<int>& sons = skel.get_sons( parent );
        assert(sons.size() > 0);
        int son0 = sons[0];

        //if(son0 != bone_id) // In factor bone mode samples are always in the first son
            //return;
        // <- yea but caps are not stored in the first son ...

        nodes = _sample_list[son0].nodes;
        n_nodes = _sample_list[son0].n_nodes;

        for( unsigned i = 0; i < sons.size(); i++) {
            int bid = sons[i];
            if(_bone_caps[bid].jcap.enable && !skel.is_leaf(bid))
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

        return son0;
    }
    else
    {
        // Concat nodes and normals
        nodes = _sample_list[bone_id].nodes;
        n_nodes = _sample_list[bone_id].n_nodes;

        // concat joint caps
        if(_bone_caps[bone_id].jcap.enable && !skel.is_leaf(bone_id))
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

        return bone_id;
    }
}
