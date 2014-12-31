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

void SampleSet::SampleSet::read_samples(std::ifstream& file,
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

void SampleSet::SampleSet::read_hrbf_env(std::ifstream& file)
{
    std::string nil;
    int bone_id  = -1;
    int nb_bones_file = 0;

    file >> nil/*'nb_bone'*/ >> nb_bones_file;

    for(int i = 0; i<nb_bones_file; i++ )
    {
        file >> nil/*'bone_id'*/ >> bone_id;
        read_samples(file, _samples[bone_id]._sample_list.nodes, _samples[bone_id]._sample_list.n_nodes);
    }
}

void SampleSet::SampleSet::choose_hrbf_samples_ad_hoc(const Animesh &animesh,
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

    _samples[bone_id]._sample_list.nodes.  clear();
    _samples[bone_id]._sample_list.n_nodes.clear();

    heur.sample(_samples[bone_id]._sample_list.nodes,
                _samples[bone_id]._sample_list.n_nodes);
}

void SampleSet::SampleSet::choose_hrbf_samples_poisson(const Animesh &animesh,
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

    _samples[bone_id]._sample_list.nodes.  clear();
    _samples[bone_id]._sample_list.n_nodes.clear();

    heur.sample(_samples[bone_id]._sample_list.nodes,
                _samples[bone_id]._sample_list.n_nodes);
}

void SampleSet::SampleSet::compute_jcaps(const Skeleton &skel, int bone_id, HSample_list &out) const
{
    const Bone* b = skel.get_bone(bone_id);

    float jrad = 0.f;// joint radius mean radius
    int nb_sons = (int) skel.get_sons(bone_id).size();
    for (int i = 0; i < nb_sons; ++i)
        jrad += _samples[ skel.get_sons(bone_id)[i] ]._junction_radius;

    jrad /= nb_sons > 0 ? (float)nb_sons : 1.f;

    Point_cu p = b->end() + b->dir().normalized() * jrad;
    Vec3_cu  n = b->dir().normalized();
    out.nodes.  push_back( p );
    out.n_nodes.push_back( n );

    //add_circle(jrad, b->get_org(), n, out_verts, out_normals);
}

void SampleSet::SampleSet::compute_pcaps(const Skeleton &skel, int bone_id,
                                 bool use_parent_dir,
                                 HSample_list &out) const
{
    const Bone* b = skel.get_bone(bone_id);
    int parent = skel.parent(bone_id);
    float prad = _samples[bone_id]._junction_radius; // parent joint radius
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
    out.nodes.push_back(p);
    out.n_nodes.push_back(n);
    //add_circle(prad, b->get_org() + b->get_dir(), n, out_verts, out_normals);
}


void SampleSet::SampleSet::update_caps(const Skeleton &skel, int bone_id, bool jcap, bool pcap)
{
    if(jcap && !skel.is_leaf(bone_id))
    {
        _samples[bone_id].jcap.samples.clear();
        compute_jcaps(skel, bone_id, _samples[bone_id].jcap.samples);
    }

    if(pcap)
    {
        int parent = skel.parent( bone_id );
        bool use_parent_dir = false;
        if(_factor_bones && parent != -1)
        {
            const std::vector<int>& sons = skel.get_sons( parent );
            assert(sons.size() > 0);
            bone_id = sons[0];
            use_parent_dir = sons.size() > 1;
        }

        _samples[bone_id].pcap.samples.clear();
        compute_pcaps(skel, bone_id, use_parent_dir, _samples[bone_id].pcap.samples);
    }
}

void SampleSet::InputSample::set_jcap(bool state)
{
    jcap.enable = state;
}

void SampleSet::InputSample::set_pcap(bool state)
{
    pcap.enable = state;
}

void SampleSet::InputSample::delete_sample(int idx)
{
    std::vector<Vec3_cu>::iterator it = _sample_list.nodes.begin();
    _sample_list.nodes  .erase( it+idx );
    it = _sample_list.n_nodes.begin();
    _sample_list.n_nodes.erase( it+idx );
}

// -----------------------------------------------------------------------------

void SampleSet::InputSample::clear()
{
    _sample_list.clear();
}

// -----------------------------------------------------------------------------

int SampleSet::InputSample::add_sample(const Vec3_cu& p, const Vec3_cu& n)
{
    _sample_list.nodes.  push_back(p);
    _sample_list.n_nodes.push_back(n);

    return (int) _sample_list.nodes.size()-1;
}

// -----------------------------------------------------------------------------

void SampleSet::InputSample::add_samples(const std::vector<Vec3_cu>& p, const std::vector<Vec3_cu>& n)
{
    assert(p.size() == n.size());

    _sample_list.nodes.insert(_sample_list.nodes.end(), p.begin(), p.end());
    _sample_list.n_nodes.insert(_sample_list.n_nodes.end(), n.begin(), n.end());
}

void SampleSet::SampleSet::write_hrbf_env(std::ofstream& file) const
{
    file << "[HRBF_ENV]" << std::endl;
    file << "nb_bone "   << _samples.size() << std::endl;

    for(unsigned i = 0; i < _samples.size(); i++ )
    {
        file << "bone_id " << i << std::endl;

        write_samples(file, _samples[i]._sample_list.nodes, _samples[i]._sample_list.n_nodes);
    }
}

void SampleSet::SampleSet::write_samples(std::ofstream& file,
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

void SampleSet::SampleSet::transform_samples(const std::vector<Transfo> &transfos, const std::vector<int>& bone_ids)
{
    int acc = 0;
    int nb_bones = int(bone_ids.size() == 0 ? _samples.size() : bone_ids.size());
    for(int i = 0; i < nb_bones; i++)
    {
        // Transform samples
        const int bone_id = bone_ids.size() == 0 ? i : bone_ids[i];
        const Transfo& tr = transfos[bone_id];

        for(unsigned j = 0; j < _samples[bone_id]._sample_list.nodes.size(); j++)
        {
            Point_cu  p = Convs::to_point(_samples[bone_id]._sample_list.nodes[j]);
            Vec3_cu n = _samples[bone_id]._sample_list.n_nodes[j];

            Vec3_cu pos = Convs::to_vector(tr * p);
            Vec3_cu nor = tr * n;
            _samples[bone_id]._sample_list.nodes  [j] = pos;
            _samples[bone_id]._sample_list.n_nodes[j] = nor;

            acc++;
        }

        transform_caps(bone_id, tr);
    }
}

void SampleSet::SampleSet::transform_caps(int bone_id, const Transfo& tr)
{
    // Transform jcaps
    if(_samples[bone_id].jcap.enable)
        for(unsigned j = 0; j < _samples[bone_id].jcap.samples.nodes.size(); j++)
        {
            Point_cu p = Convs::to_point(_samples[bone_id].jcap.samples.nodes[j]);
            Vec3_cu  n = _samples[bone_id].jcap.samples.n_nodes[j];
            _samples[bone_id].jcap.samples.nodes  [j] = Convs::to_vector(tr * p);
            _samples[bone_id].jcap.samples.n_nodes[j] = tr * n;
        }

    // Transform pcaps
    if(_samples[bone_id].pcap.enable)
        for(unsigned j = 0; j < _samples[bone_id].pcap.samples.nodes.size(); j++)
        {
            Point_cu p = Convs::to_point(_samples[bone_id].pcap.samples.nodes[j]);
            Vec3_cu  n = _samples[bone_id].pcap.samples.n_nodes[j];
            _samples[bone_id].pcap.samples.nodes  [j] = Convs::to_vector(tr * p);
            _samples[bone_id].pcap.samples.n_nodes[j] = tr * n;
        }
}

int SampleSet::SampleSet::compute_nb_samples() const
{
    int acc = 0;
    for(unsigned i = 0; i < _samples.size(); i++)
        acc += (int) _samples[i]._sample_list.nodes.size();

    return acc;
}

int SampleSet::SampleSet::get_all_bone_samples(const Skeleton &skel, int bone_id, HSample_list &out) const
{
    std::vector<int> sons;
    if( _factor_bones && bone_id != skel.root() )
    {
        int parent = skel.parent( bone_id );
        sons = skel.get_sons( parent );
        assert(sons.size() > 0);
        bone_id = sons[0];
    }
    else
    {
        sons.push_back(bone_id);
    }

    // Concat nodes and normals
    out.append(_samples[bone_id]._sample_list);

    // concat joint caps
    for( unsigned i = 0; i < sons.size(); i++) {
        int bid = sons[i];
        if(!_samples[bid].jcap.enable || skel.is_leaf(bid))
            continue;

        out.append(_samples[bid].jcap.samples);
    }

    // concat parent caps
    if(_samples[bone_id].pcap.enable) {
        out.append(_samples[bone_id].pcap.samples);
    }

    return bone_id;
}
