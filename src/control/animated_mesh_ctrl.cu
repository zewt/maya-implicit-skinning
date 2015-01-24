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

// -----------------------------------------------------------------------------
namespace { __device__ void fix_debug() { } }

// This takes ownership of the Mesh (and gives it to Animesh).  It does not take ownership
// of Skeleton.
Animated_mesh_ctrl::Animated_mesh_ctrl(const Mesh *mesh, std::shared_ptr<const Skeleton> skel_) :
    skel(skel_),
    _animesh(new Animesh(mesh, skel_))
{
}

Animated_mesh_ctrl::~Animated_mesh_ctrl()
{
    delete _animesh;
}

void Animated_mesh_ctrl::set_do_smoothing(bool state) { _animesh->set_smooth_mesh(state); }
void Animated_mesh_ctrl::set_smooth_factor(int i, float fact) { _animesh->set_smooth_factor(i, fact); }
void Animated_mesh_ctrl::update_base_potential() { _animesh->update_base_potential(); }
void Animated_mesh_ctrl::get_base_potential(std::vector<float> &pot) const  { _animesh->get_base_potential(pot); }
void Animated_mesh_ctrl::set_base_potential(const std::vector<float> &pot) { _animesh->set_base_potential(pot); }
void Animated_mesh_ctrl::deform_mesh() { _animesh->transform_vertices(); }
void Animated_mesh_ctrl::set_nb_iter_smooting(int nb_iter) { _animesh->set_smoothing_iter(nb_iter); }
void Animated_mesh_ctrl::smooth_conservative() { _animesh->set_smoothing_type(EAnimesh::CONSERVATIVE); }
void Animated_mesh_ctrl::smooth_laplacian() { _animesh->set_smoothing_type(EAnimesh::LAPLACIAN); }
void Animated_mesh_ctrl::smooth_tangential() { _animesh->set_smoothing_type(EAnimesh::TANGENTIAL); }
void Animated_mesh_ctrl::smooth_humphrey() { _animesh->set_smoothing_type(EAnimesh::HUMPHREY); }
void Animated_mesh_ctrl::set_local_smoothing(bool state) { _animesh->set_local_smoothing(state); }
void Animated_mesh_ctrl::set_smooth_force_a (float alpha) { _animesh->set_smooth_force_a(alpha); }
void Animated_mesh_ctrl::set_smooth_force_b (float beta) { _animesh->set_smooth_force_b(beta); }
void Animated_mesh_ctrl::set_smooth_smear(float val ) { _animesh->set_smooth_smear(val); }
void Animated_mesh_ctrl::set_smoothing_weights_diffusion_iter(int nb_iter) { _animesh->set_smoothing_weights_diffusion_iter(nb_iter); }
void Animated_mesh_ctrl::get_anim_vertices_aifo(std::vector<Point_cu>& out) const { _animesh->get_anim_vertices_aifo(out); }
void Animated_mesh_ctrl::copy_vertices(const std::vector<Vec3_cu> &vertices) { _animesh->copy_vertices(vertices); }
int Animated_mesh_ctrl::get_nb_vertices() const { return _animesh->get_nb_vertices(); }
