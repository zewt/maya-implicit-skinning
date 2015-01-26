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
#include "skeleton.hpp"
#include "animesh_hrbf_heuristic.hpp"

#include <sstream>

void SampleSet::SampleSet::choose_hrbf_samples(const Mesh *mesh, const Skeleton *skel, const VertToBoneInfo &vertToBoneInfo, const SampleSetSettings &settings, int bone_id)
{
    if(!skel->is_bone(bone_id))
        return;

    if(settings.mode == SampleSetSettings::AdHoc)
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

    // Don't add caps if we don't have any actual samples.
    if(_samples[bone_id].nodes.empty())
        return;

    if(settings.jcap)
        compute_jcaps(*skel, settings, bone_id, _samples[bone_id]);

    if(settings.pcap)
        compute_pcaps(*skel, settings, bone_id, _samples[bone_id]);
}

void SampleSet::SampleSet::compute_jcaps(const Skeleton &skel, const SampleSetSettings &settings, int bone_id, InputSample &out) const
{
    if(skel.is_leaf(bone_id))
        return;

    const Bone* b = skel.get_bone(bone_id).get();

    // Get the average _junction_radius of the joint's children.
    float jrad = 0.f;// joint radius mean radius
    const std::vector<int> &children = skel.get_sons(bone_id);
    int count = 0;

    // Ignore children that are leaves.  
    // We're assuming that most bones are cylindrical (arms, fingers), and looking for the radius
    // of the cylinder where it joints with children.  The junction_radius for a bone is its minimum
    // distance from the bone to any of its vertices, and we'll take the average of those to guess
    // the radius.
    //
    // This assumes that the "caps" of each cylinder have effectively been chopped off during
    // clusterization, but that's not the case for leaf joints: the end of a finger will still
    // be there.  The last joint of a finger often sits right at the fingertip, which means that
    // the radius of that last joint will be 0, which isn't what we want.  Work around this by
    // ignoring leaf joints, and if we end up with no joints at all, use the parent joint's radius
    // instead.  This isn't perfect, but seems to help.
    for (int i = 0; i < (int) children.size(); ++i) {
        int child_bone_id = children[i];
        if(skel.is_leaf(child_bone_id))
            continue;

        ++count;
        jrad += settings.junction_radius.at(child_bone_id);
    }

    if(count == 0)
        jrad = settings.junction_radius.at(bone_id);
    else
        jrad /= count;

    Point_cu p = b->end() + b->dir().normalized() * jrad;
    Vec3_cu  n = b->dir().normalized();
    out.nodes.  push_back( p );
    out.n_nodes.push_back( n );
}

void SampleSet::SampleSet::compute_pcaps(const Skeleton &skel, const SampleSetSettings &settings, int bone_id,
                                 InputSample &out) const
{
    const Bone* b = skel.get_bone(bone_id).get();
    float prad = settings.junction_radius.at(bone_id);
    Vec3_cu p =  b->org() - b->dir().normalized() * prad;
    out.nodes.push_back(p);

    Vec3_cu n = -b->dir().normalized();
    out.n_nodes.push_back(n);
}


void SampleSet::InputSample::delete_sample(int idx)
{
    std::vector<Vec3_cu>::iterator it = nodes.begin();
    nodes  .erase( it+idx );
    it = n_nodes.begin();
    n_nodes.erase( it+idx );
}

void SampleSet::InputSample::transform(const Transfo &matrix)
{
    for(int i = 0; i < nodes.size(); ++i)
    {
        // Nodes are vec3, but they're really points.  The difference is that we do want the points
        // to be translated, where the normals are only rotated and scaled but not translated.
        // Rather than changing the data type in a ton of places, for now just apply the matrix
        // as if it was a point.
        nodes[i] = matrix.multiply_as_point(nodes[i]);
        n_nodes[i] = matrix * n_nodes[i];
    }
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
            Point_cu  p = _samples[bone_id].nodes[j].to_point();
            Vec3_cu n = _samples[bone_id].n_nodes[j];

            Vec3_cu pos = (tr * p).to_vector();
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
