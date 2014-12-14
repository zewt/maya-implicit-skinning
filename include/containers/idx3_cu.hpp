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
#ifndef IDX3_CU_HPP
#define IDX3_CU_HPP

#include "cuda_compiler_interop.hpp"
#include "vec3i_cu.hpp"

/**
 * @struct Idx3_cu
 * @brief 3d grid index with conversion to 3d and linear storage
 *
 * Use case to look up every elements of a grid of 16x16x16
 * @code
 *
 * Vec3i_cu s(16, 16, 16);
 * for(Idx3_cu idx(s, 0); idx.is_in(); ++idx)
 * {
 *      Vec3i_cu idx_3d = idx.to_3d(); // or .to_vec3i() will do the same
 *      int      i      = idx.to_linear();
 *
 *      // i == idx_3d.x + s.x * idx_3d.y + (s.x * s.y) * idx_3d.z
 * }
 * @endcode
 *
 * Looking up a sub grid 10^3 inside a grid 128^3:
 * @code
 * Vec3i_cu sub_grid_s(10, 10, 10);
 * Vec3i_cu grid_s(128, 128, 128);
 * Idx3_cu offset(grid_s, offx, offy, offz);
 * for(Idx3_cu idx(sub_grid_s, 0); idx.is_in(); ++idx)
 * {
 *     int i = (offset + idx.to_vec3i()).to_linear();
 *     // 'i' is in linear coordinates of the grid 128^3 but will only look up
 *     // a sub block of size 10^3 starting from the offset in the 128^3 grid
 * }
 * @endcode
 */

struct Idx3_cu {


    // -------------------------------------------------------------------------
    /// @name Constructors
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline
    Idx3_cu() :  _size(-1, -1, -1), _id(-1) { }

    /// Build index from a linear index
    IF_CUDA_DEVICE_HOST inline
    Idx3_cu(const Vec3i_cu& size, int idx) : _size(size), _id(idx) { }

    /// Build index from a 3d index
    IF_CUDA_DEVICE_HOST inline
    Idx3_cu(const Vec3i_cu& size, int ix, int iy, int iz) : _size(size) {
        _id = to_linear(_size, ix, iy, iz);
    }

    /// Build index from a 3d index
    IF_CUDA_DEVICE_HOST inline
    Idx3_cu(const Vec3i_cu& size, const Vec3i_cu& pos) : _size(size) {
        set_3d( pos );
    }

    // -------------------------------------------------------------------------
    /// @name Set index position
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline
    void set_linear(int i){ _id = i; }

    IF_CUDA_DEVICE_HOST inline
    void set_3d(const Vec3i_cu& p){ set_3d(p.x, p.y, p.z); }

    IF_CUDA_DEVICE_HOST inline
    void set_3d(int x, int y, int z){ _id = to_linear(_size, x, y, z); }

    IF_CUDA_DEVICE_HOST inline
    int to_linear() const { return _id; }

    // -------------------------------------------------------------------------
    /// @name Get index position
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline
    Vec3i_cu to_3d() const { Vec3i_cu r; to_3d(r.x, r.y, r.z); return r; }

    IF_CUDA_DEVICE_HOST inline
    Vec3i_cu to_vec3i() const { return to_3d(); }

    IF_CUDA_DEVICE_HOST inline
    void to_3d(int& x, int& y, int& z) const {
        x = _id % _size.x;
        z = _id / _size.x;
        y =  z  % _size.y;
        z =  z  / _size.y;
    }

    // -------------------------------------------------------------------------
    /// @name Other methods
    // -------------------------------------------------------------------------

#ifdef __CUDACC__
    int4 to_int4() const { return make_int4(_size.x, _size.y, _size.z, _id); }
#endif

    IF_CUDA_DEVICE_HOST inline
    int size_linear() const { return _size.product(); }

    IF_CUDA_DEVICE_HOST inline
    Vec3i_cu size() const { return _size; }

    /// A valid index is positive as well as its size
    IF_CUDA_DEVICE_HOST inline
    bool is_valid() const {
        return _id >= 0 && size_linear() >= 0;
    }

    /// Does the index is out of its bounds (defined at construction)
    IF_CUDA_DEVICE_HOST inline bool is_out() const { return !is_in(); }

    /// Does the index is inside its bounds (defined at construction)
    IF_CUDA_DEVICE_HOST inline
    bool is_in() const {
        return (_id < size_linear()) && (_id >= 0);
    }

    // -------------------------------------------------------------------------
    /// @name Operators overload
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline
    Idx3_cu operator++(   ) { return Idx3_cu(_size, ++_id); }

    IF_CUDA_DEVICE_HOST inline Idx3_cu operator++(int)
    { return Idx3_cu(_size, _id++); }

    IF_CUDA_DEVICE_HOST inline
    Idx3_cu operator--(   ) { return Idx3_cu(_size, --_id); }

    IF_CUDA_DEVICE_HOST inline
    Idx3_cu operator--(int) { return Idx3_cu(_size, _id--); }

    IF_CUDA_DEVICE_HOST inline
    bool operator==(const Idx3_cu& i) const {
        return _size == i._size && _id == i._id;
    }

    IF_CUDA_DEVICE_HOST inline
    bool operator!=(const Idx3_cu& i) const {
        return _size != i._size || _id != i._id;
    }

    IF_CUDA_DEVICE_HOST inline
    Idx3_cu operator =(const Idx3_cu& i) {
        _size = i._size; _id = i._id; return *this;
    }

    IF_CUDA_DEVICE_HOST inline friend
    Idx3_cu operator+ (const Idx3_cu& id, const Vec3i_cu& v) {
        Vec3i_cu this_idx = id.to_3d();
        return Idx3_cu(id._size, this_idx + v);
    }

    IF_CUDA_DEVICE_HOST inline friend
    Idx3_cu operator+ (const Vec3i_cu& v, const Idx3_cu& id) {
        return id + v;
    }

private:

    IF_CUDA_DEVICE_HOST static inline
    int to_linear(const Vec3i_cu& size, int x, int y, int z) {
        return x + size.x * (y + size.y * z);
    }

    Vec3i_cu _size; ///< 3d size the index is looking up
    int      _id;   ///< Linear index

    // WARNING: these operators should not be used since :
    // (they don't really make sense) || (are to ambigus to decypher when used)
#if 0
    bool operator<=(const Idx3_cu& ) const { return false; }
    bool operator>=(const Idx3_cu& ) const { return false; }
    bool operator< (const Idx3_cu& ) const { return false; }
    bool operator> (const Idx3_cu& ) const { return false; }

    Idx3_cu operator- (const Idx3_cu& ) const { return Idx3_cu(); }
    Idx3_cu operator+ (const Idx3_cu& ) const { return Idx3_cu(); }
    Idx3_cu operator+=(const Idx3_cu& )       { return Idx3_cu(); }
    Idx3_cu operator-=(const Idx3_cu& )       { return Idx3_cu(); }

    bool operator==(int ) const { return false; }
    bool operator!=(int ) const { return false; }
    bool operator<=(int ) const { return false; }
    bool operator>=(int ) const { return false; }
    bool operator> (int ) const { return false; }
    bool operator< (int ) const { return false; }

    Idx3_cu operator+ (int )  const { return Idx3_cu(); }
    Idx3_cu operator- (int )  const { return Idx3_cu(); }
    Idx3_cu operator+=(int )        { return Idx3_cu(); }
    Idx3_cu operator-=(int )        { return Idx3_cu(); }
#endif
};

#endif // IDX3_CU_HPP
