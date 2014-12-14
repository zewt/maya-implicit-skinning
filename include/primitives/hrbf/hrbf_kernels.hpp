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
#ifndef HRBF_KERNELS_HPP__
#define HRBF_KERNELS_HPP__

#include "cuda_utils.hpp"
#include "transfo.hpp"

// =============================================================================
namespace HRBF_kernels{
// =============================================================================

/// Transform each vertex of each rbf primitive
/// @param d_transform : transformations associated to each HRBF instances
/// d_transform[hrbf_id] = transfo_instance
/// @param d_map_transfos : Mapping of hrbf points with their transformations
/// in d_transform.
/// d_map_transfos[hrbf_id_offset + sample_idx] = d_transfo[hrbf_id]
void hrbf_transform(const Cuda_utils::Device::Array<Transfo>& d_transform,
                    const Cuda_utils::DA_int& d_map_transfos);

}// END HRBF_ENV NAMESPACE =====================================================

#endif // HRBF_KERNELS_HPP__
