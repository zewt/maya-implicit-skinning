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
#include "conversions.hpp"
#include "skeleton.hpp"
#include "animesh_hrbf_heuristic.hpp"

#include <sstream>

void SampleSet::SampleSet::choose_hrbf_samples(const Mesh *mesh, const Skeleton *skel, const VertToBoneInfo &vertToBoneInfo, const SampleSetSettings &settings, int bone_id)
{
    if(!skel->is_bone(bone_id))
        return;

    if(settings.mode == SampleSetSettings::Poisson)
    {
        Adhoc_sampling heur(mesh, skel, vertToBoneInfo);
        heur._bone_id = bone_id;
        heur._jmax = settings.jmax;
        heur._pmax = settings.pmax;
        heur._mind = settings.min_dist;
        heur._fold = settings.fold;
        heur._factor_siblings = false;

        _samples[bone_id].nodes.  clear();
        _samples[bone_id].n_nodes.clear();

        heur.sample(_samples[bone_id].nodes,
                    _samples[bone_id].n_nodes);
    }
    else
    {
        Poisson_disk_sampling heur(mesh, skel, vertToBoneInfo);
        heur._bone_id = bone_id;
        heur._jmax = settings.jmax;
        heur._pmax = settings.pmax;
        heur._mind = settings.min_dist;
        heur._nb_samples = settings.nb_samples;
        heur._fold = settings.fold;
        heur._factor_siblings = false;

        _samples[bone_id].nodes.  clear();
        _samples[bone_id].n_nodes.clear();

        heur.sample(_samples[bone_id].nodes, _samples[bone_id].n_nodes);
    }

    if(settings.jcap)
        compute_jcaps(*skel, settings, bone_id, _samples[bone_id]);

    if(settings.pcap)
    {
        int parent = skel->parent( bone_id );
        bool use_parent_dir = false;
        compute_pcaps(*skel, settings, bone_id, use_parent_dir, _samples[bone_id]);
    }
}

void SampleSet::SampleSet::compute_jcaps(const Skeleton &skel, const SampleSetSettings &settings, int bone_id, InputSample &out) const
{
    if(skel.is_leaf(bone_id))
        return;

    const Bone* b = skel.get_bone(bone_id);

    // Get the average _junction_radius of the joint's children.
    float jrad = 0.f;// joint radius mean radius
    const std::vector<int> &children = skel.get_sons(bone_id);
    int nb_sons = (int) children.size();
    for (int i = 0; i < nb_sons; ++i)
        jrad += settings.junction_radius.at(children[i]);

    // Note that there should never actually be zero children, because our caller checks.
    jrad /= nb_sons > 0 ? (float)nb_sons : 1.f;

    Point_cu p = b->end() + b->dir().normalized() * jrad;
    Vec3_cu  n = b->dir().normalized();
    out.nodes.  push_back( p );
    out.n_nodes.push_back( n );

    //add_circle(jrad, b->get_org(), n, out_verts, out_normals);
}

void SampleSet::SampleSet::compute_pcaps(const Skeleton &skel, const SampleSetSettings &settings, int bone_id,
                                 bool use_parent_dir,
                                 InputSample &out) const
{
    const Bone* b = skel.get_bone(bone_id);
    int parent = skel.parent(bone_id);
    float prad = settings.junction_radius.at(bone_id); // parent joint radius
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


void SampleSet::InputSample::delete_sample(int idx)
{
    std::vector<Vec3_cu>::iterator it = nodes.begin();
    nodes  .erase( it+idx );
    it = n_nodes.begin();
    n_nodes.erase( it+idx );
}

// -----------------------------------------------------------------------------

void SampleSet::InputSample::clear()
{
    nodes.clear();
    n_nodes.clear();
}

void SampleSet::InputSample::append(const InputSample &rhs)
{
    nodes.insert(nodes.end(), rhs.nodes.begin(), rhs.nodes.end());
    n_nodes.insert(n_nodes.end(), rhs.n_nodes.begin(), rhs.n_nodes.end());
}

int SampleSet::InputSample::add_sample(const Vec3_cu& p, const Vec3_cu& n)
{
    nodes.  push_back(p);
    n_nodes.push_back(n);

    return (int) nodes.size()-1;
}

// -----------------------------------------------------------------------------

void SampleSet::InputSample::add_samples(const std::vector<Vec3_cu>& p, const std::vector<Vec3_cu>& n)
{
    assert(p.size() == n.size());

    nodes.insert(nodes.end(), p.begin(), p.end());
    n_nodes.insert(n_nodes.end(), n.begin(), n.end());
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

        for(unsigned j = 0; j < _samples[bone_id].nodes.size(); j++)
        {
            Point_cu  p = Convs::to_point(_samples[bone_id].nodes[j]);
            Vec3_cu n = _samples[bone_id].n_nodes[j];

            Vec3_cu pos = Convs::to_vector(tr * p);
            Vec3_cu nor = tr * n;
            _samples[bone_id].nodes  [j] = pos;
            _samples[bone_id].n_nodes[j] = nor;

            acc++;
        }
    }
}

void SampleSet::SampleSet::get_all_bone_samples(Bone::Id bone_id, InputSample &out) const
{
    out.append(_samples.at(bone_id));
}
