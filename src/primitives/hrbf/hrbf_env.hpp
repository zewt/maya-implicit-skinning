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
#ifndef HRBF_ENV_HPP__
#define HRBF_ENV_HPP__

// -----------------------------------------------------------------------------

#include "cuda_utils.hpp"
#include "transfo.hpp"

// -----------------------------------------------------------------------------

#include <fstream>
#include <vector>

// -----------------------------------------------------------------------------

struct Skeleton;

// -----------------------------------------------------------------------------

/**
  @namespace HRBF_env
  @brief Environment to store and manage instances of hermite RBF into CUDA
  texture memory

  This interface enable to transfert hermite rbf data into texture memory.
  Once data is stored fecthing is done throught fetch_xxx() functions in
  HRBF_Env::rbf_hermite_tex().

  You can add/delete/move points.

  usage:
  @code


  @endcode

  @warning when accessing global arrays of this namespace you must unbind them
  before any changes and then rebind them. You can do so with bind() and
  unbind() functions from HRBF_Env namespace in "rbf_hermite_tex_interface.hpp".
*/
// TODO: use typedefs for hrbf id
// =============================================================================
namespace HRBF_env{
// =============================================================================

/// Animated points and weights.
/// These arrays represents the mesh when animated
/// There are computed by the function rbf_transform()
#if !defined(NO_CUDA)
extern Cuda_utils::HDA_float4 hd_points;
/// First three floats represents the beta vector, last float is the alpha scalar
extern Cuda_utils::HDA_float4 hd_alphas_betas;

/// initial vertices sorted by bones
extern Cuda_utils::DA_float4 d_init_points;
/// initial weights
extern Cuda_utils::DA_float4 d_init_alpha_beta;

/// Each hrbf instance posses an id corresponding to an index of d_offset.
/// d_offset array stores for each hermiteRBF ID the offset needed
/// to acces the hermiteRBF's coefficients (d_alphas_betas[] array)
/// and points (d_points[] array). d_offset[HRBF_ID].x = offset
/// d_offset[HRBF_ID].y == nb_samples_hrbf
/// d_offset[is].x is the prefix sum of HRBF instances size.
/// When the offset is negative this means there is no instance associated
/// to the id
extern Cuda_utils::DA_int2 d_offset;
extern Cuda_utils::HA_int2 h_offset;

extern Cuda_utils::HA_Vec3_cu h_normals;

/// Radius of the hrbfs to transform from global to compact support.
extern Cuda_utils::HDA_float hd_radius;
#endif

// -----------------------------------------------------------------------------

void bind();
void unbind();

/// delete every instances from memory and unbind texture delete vbos as well
/// call this when you quit the app
void clean_env();

/// If you wish to erase every hrbf and refill the env use this
void reset_env();

//------------------------------------------------------------------------------
/// @name Manage HRBF instances
//------------------------------------------------------------------------------

/// @return  the instance id
int new_instance();

/// Deletes the instance of id 'hrbf_id'
void delete_instance(int hrbf_id);

/// Empty the hrbf instance of id hrbf_id
void reset_instance(int hrbf_id);

//------------------------------------------------------------------------------
/// @name Handling instance datas
//------------------------------------------------------------------------------

/// delete a samples designated by the vector "samples_idx" from the hrbf
/// instance of id "hrbf_id". The weights are updated with the new samples
/// @warning this operation could take a while..
void delete_samples(int hrbf_id, const std::vector<int>& samples_idx);

/// Function shortcut to delete a single sample
void delete_sample(int hrbf_id, int sample_idx);

/// add samples to a hrbf instance of id "hrbf_id". The weights are re-computed
/// with the new samples or copy from the array 'weights'
/// @param hrbf_id index of the hrbf from which you want to add samples
/// @param points position of the new samples
/// @param normals Vector of normals for the new samples
/// @param weights the coefficients bounds to each sample. First three floats
/// represents the beta vector, last float is the alpha scalar. If 'weights' is
/// the zero sized vector then coefficients are computed again ignoring this
/// vector
/// @return the index of the first sample
/// @warning this operation could take a while as hrbf weights has to be
/// computed
#if !defined(NO_CUDA)
int add_samples(int hrbf_id,
                const std::vector<Vec3_cu>& points,
                const std::vector<Vec3_cu>& normals,
                const std::vector<float4>& weights = std::vector<float4>(0));
#endif

/// Function shortcut to add a single sample
/// @return the index of the sample
int add_sample(int hrbf_id, const Vec3_cu& point, const Vec3_cu& normal);

//------------------------------------------------------------------------------
/// @name Setters
//------------------------------------------------------------------------------

/// set a new position for the ith point
void set_sample(int hrbf_id,
                int sample_index,
                const Vec3_cu& p);

/// set a new normal for the ith point
void set_sample_normal(int hrbf_id,
                       int sample_index,
                       const Vec3_cu& n);

/// Set the radius of the ith instance for going to global to compact support
void set_inst_radius(int hrbf_id, float radius);

/// Set transformations of the ith instance which will be used to compute
/// Animated samples and weights of the HRBF
/// @warning to apply the transformation call apply_hrbf_transfos()
/// when all transformations are sets
void set_transfo(int hrbf_id, const Transfo& tr);

/// Apply HRBF transformations defined with set_transfo(); to every hrbf
/// instances
// TODO: only apply transformations to a specific HRBF
void apply_hrbf_transfos();

//------------------------------------------------------------------------------
/// @name Getters
//------------------------------------------------------------------------------

/// Get the list of current instances ids
/// @param list_id The list of HRBF ids.
/// The vector is cleared before pushing the ids
void get_instance_id_list(std::vector<int>& list_id);

/// Number of points of the ith hrbf instance
int get_instance_size(int i);

/// get position of the sample in rest position of index 'sample_idx' which
/// belongs to the instance hrbf_id
Point_cu get_sample(int hrbf_id, int sample_idx);

/// Get the list of samples that belongs to the instance hrbf_id.
/// 'samp_list' will be resized to match get_instance_size(hrbf_id) and
/// filled with the animated position of the samples.
void get_anim_samples(int hrbf_id, std::vector<Point_cu>& samp_list);

/// Get the list of weights that belongs to the instance hrbf_id.
/// 'weights_list' will be resized to match get_instance_size(hrbf_id) and
/// filled with the animated weights. Note: (x, y ,z) = beta_weight w = alpha_weight
#if !defined(NO_CUDA)
void get_anim_weights(int hrbf_id, std::vector<float4>& weights_list);
#endif

/// Get the list of samples that belongs to the instance hrbf_id.
/// 'samp_list' will be resized to match get_instance_size(hrbf_id) and
/// filled with the initial position of the samples.
void get_samples(int hrbf_id, std::vector<Vec3_cu>& samp_list);

/// Get the list of weights that belongs to the instance hrbf_id.
/// 'weights_list' will be resized to match get_instance_size(hrbf_id) and
/// filled with the initial weights. Note: (x, y ,z) = beta_weight w = alpha_weight
#if !defined(NO_CUDA)
void get_weights(int hrbf_id, std::vector<float4>& weights_list);
#endif

/// get normal in initial postion
Vec3_cu get_normal(int hrbf_id, int sample_idx);

/// get animated normal
Vec3_cu get_anim_normal(int hrbf_id, int sample_idx);

/// Get the list of normals that belongs to the instance hrbf_id.
/// 'normal_list' will be resized to match get_instance_size(hrbf_id) and
/// filled with the initial direction of the samples' normals.
void get_normals(int hrbf_id, std::vector<Vec3_cu>& normal_list);

/// Get the list of normals that belongs to the instance hrbf_id.
/// 'normal_list' will be resized to match get_instance_size(hrbf_id) and
/// filled with the animated direction of the samples' normals.
/// We use get_normals() normals and apply the inverse transpose.
void get_anim_normals(int hrbf_id, std::vector<Vec3_cu>& normal_list);

#if !defined(NO_CUDA)
/// get weights
float4 get_weights(int hrbf_id, int sample_idx);
#endif

/// Get the radius of the hrbf instance. (radius to go from global support to
/// compact)
float get_inst_radius(int hrbf_id);

/// Get transformations of the ith instance
Transfo get_transfo(int hrbf_id);


/// @return the instance radius to transform from global to compact support
IF_CUDA_DEVICE_HOST static inline
float fetch_radius(int id_instance);

/// @return the instance offset in x and size in y
#if !defined(NO_CUDA)
IF_CUDA_DEVICE_HOST static inline
int2 fetch_inst_size_and_offset(int id_instance);
#endif

/// fetch at the same time points and weights (more efficient than functions below)
/// @param raw_idx The raw index to fetch directly from the texture.
/// raw_idx equals the offset plus the indice of the fetched sample
/// @return the alpha weight
IF_CUDA_DEVICE_HOST inline static
float fetch_weights_point(Vec3_cu& beta,
                          Point_cu& point,
                          int raw_idx);

}// END HRBF_ENV NAMESPACE =====================================================

#if !defined(NO_CUDA)
#include "hrbf_env.inl"
#endif

#endif // HRBF_ENV_HPP__
