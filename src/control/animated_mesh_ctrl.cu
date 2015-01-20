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
#include "std_utils.hpp"
#include "skeleton.hpp"
#include "timer.hpp"
#include "precomputed_prim.hpp"
#include "hrbf_env.hpp"

// -----------------------------------------------------------------------------
namespace { __device__ void fix_debug() { } }

// This takes ownership of the Mesh (and gives it to Animesh).  It does not take ownership
// of Skeleton.
Animated_mesh_ctrl::Animated_mesh_ctrl(const Mesh *mesh, std::shared_ptr<const Skeleton> skel_) :
    _nb_iter(7),
    skel(skel_),
    _animesh(new Animesh(mesh, skel_))
{
}

Animated_mesh_ctrl::~Animated_mesh_ctrl()
{
    delete _animesh;
}

void Animated_mesh_ctrl::set_do_smoothing(bool state)
{
    if(_animesh != 0){
        _animesh->set_smooth_mesh(state);
    }
}

void Animated_mesh_ctrl::set_smooth_factor(int i, float fact){
    _animesh->set_smooth_factor(i, fact);
}

void Animated_mesh_ctrl::update_base_potential()
{
    assert(_animesh != 0);
    _animesh->update_base_potential();
}

void Animated_mesh_ctrl::get_base_potential(std::vector<float> &pot, std::vector<Vec3_cu> &grad) const  { assert(_animesh != NULL); _animesh->get_base_potential(pot, grad); }
void Animated_mesh_ctrl::set_base_potential(const std::vector<float> &pot, const std::vector<Vec3_cu> &grad) { assert(_animesh != NULL); _animesh->set_base_potential(pot, grad); }

// -----------------------------------------------------------------------------

void Animated_mesh_ctrl::deform_mesh()
{
    _animesh->transform_vertices();
}

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

void Animated_mesh_ctrl::load_ism(const char* filename)
{
    using namespace std;
    ifstream file(filename);
/*
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
        else
        {
            std::cerr << "WARNING ism import: can't read this symbol '";
            std::cerr << section << "'" << std::endl;
        }
    }
    file.close();

    _animesh->update_base_potential();
    */
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
