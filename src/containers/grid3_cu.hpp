#ifndef GRID3_CU_HPP
#define GRID3_CU_HPP

#include <vector>
#include "idx3_cu.hpp"
#include "vec3i_cu.hpp"
#include "cuda_compiler_interop.hpp"

/** @class Grid3_cu
  * @brief utility to store/use 3d grids with cuda
  * @tparam T : Cell type. Default constructor MUST be defined.
  */
template<class T>
struct Grid3_cu {

    /// @brief Padding kind enumerant.
    enum Pad_t {
        COPY,   ///< padding will extend border values
        CUSTOM  ///< padding will initialize to specified value
    };

    Grid3_cu(int x, int y, int z,
             const T* vals,
             const Vec3i_cu& pad = Vec3i_cu::zero()) :
        _size(Vec3i_cu(x, y, z)),
        _pad_off(pad)
    {
        init_vals(vals);
    }

    Grid3_cu(const Vec3i_cu& s,
             const T* vals,
             const Vec3i_cu &pad = Vec3i_cu::zero()) :
        _size(s),
        _pad_off(pad)
    {
        init_vals(vals);
    }

    Grid3_cu( const Grid3_cu<T>& g) :
        _size(g._size),
        _pad_off(g._pad_off)
    {
        init_vals(g._vals.data());
    }

    /// Here, final dimensions are computed from the maximm dimensions allowed
    /// @param list : list of 3d grids to be concatenated. All grids MUST be
    /// of the same size.
    /// @param max_x, max_y, max_z : Maximum size of the new grid resulting from
    /// the concatenation of the grid list.
    /// @param out_idx : returned indices in the new grid to access first
    /// non-padded data for each input grid.
    /// The new grid is filled if necessary but not padded.
    Grid3_cu(const std::vector< Grid3_cu<T>* >& list,
             const Vec3i_cu& max,
             std::vector<Idx3_cu>& out_idx);

    /// Build a padded grid given
    /// @param type : padding kind.
    /// @param val  : padding value when custom padding, default is T().
    /// @note the padding will be done so as to keep the old grid
    /// at the center of the new one.
    void padd(const Vec3i_cu& padding, Pad_t type = COPY, T val = T() );

    Vec3i_cu size() const { return _size; }

    Vec3i_cu get_padd_offset() const { return _pad_off; }

    const std::vector<T>& get_vals() const { return _vals; }

    /// Allocate and copy grid to GPU for future use with cuda textures
    /// @return pointer to newly allocated device mem.
    cudaArray* to_gpu() const;

private:
    void init_vals(const T *vals);

    Vec3i_cu _size;        ///< 3d size of the grid (nb elts)
    Vec3i_cu _pad_off;     ///< padding offsets (nb elts)
    std::vector<T> _vals;  ///< Linear storage of the 3D grid
};

#include "grid3_cu.inl"

#endif // GRID3_CU_HPP
