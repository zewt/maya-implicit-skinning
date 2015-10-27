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
