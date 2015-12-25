#include <vector>
#include "maths/vec3_cu.hpp"

// =============================================================================
namespace Utils_sampling {
// =============================================================================

/// @param radius : minimal radius between every samples. If radius <= 0
/// a new radius is approximated given the targeted number of samples
/// 'nb_samples'
/// @param nb_samples : ignored if radius > 0 otherwise will try to match
/// and find  nb_samples by finding an appropriate radius.
/// @param verts : list of vertices
/// @param nors : list of normlas coresponding to each verts[].
/// @param tris : triangle indices in verts[] array.
/// @code
///     tri(v0;v1;v2) = { verts[ tris[ith_tri*3 + 0] ],
///                       verts[ tris[ith_tri*3 + 1] ],
///                       verts[ tris[ith_tri*3 + 2] ]   }
/// @endcode
/// @param samples_pos : resulting samples positions
/// @param samples_nors : resulting samples normals associated to samples_pos[]
/// @warning undefined behavior if (radius <= 0 && nb_samples == 0) == true
void poisson_disk(float radius,
                  int nb_samples,
                  const std::vector<Vec3_cu>& verts,
                  const std::vector<Vec3_cu>& nors,
                  const std::vector<int>& tris,
                  std::vector<Vec3_cu>& samples_pos,
                  std::vector<Vec3_cu>& samples_nor);

}// END UTILS_SAMPLING NAMESPACE ===============================================
