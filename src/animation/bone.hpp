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
#ifndef BONE_HPP__
#define BONE_HPP__

#include <cassert>
#include <memory>
#include "cuda_compiler_interop.hpp"
#include "point_cu.hpp"
#include "bbox.hpp"
#include "bone_type.hpp"

/**
  @class Bone_cu
  @brief Mother class of various bones types

  A bone defines a segment between two 3D points.
  Attribute '_org' defines the starting point of the segment.

  The second ending point can be computed with the direction of the bone _dir
  whose magnitude equals the lenght of the bone.

  Bones points toward the skeleton leaves. Usualy bones at the leaves are zero
  length

  @code                       ->
       _org           _dir   (_org + _dir)
         +-------------->---------+
      joint                    son_joint (if any)
  @endcode

  Usually bones are associated to an implicit primitive. Thus this class is
  often subclass.
*/
class Bone_cu {
public:
    IF_CUDA_DEVICE_HOST
    Bone_cu():
        _org(0.f, 0.f, 0.f),
        _dir(0.f, 0.f, 0.f),
        _length(0)
    { }

    IF_CUDA_DEVICE_HOST
    Bone_cu(const Point_cu& p1, const Point_cu& p2):
        _org(p1),
        _dir(p2-p1),
        _length((p2-p1).norm())
    { }

    IF_CUDA_DEVICE_HOST
    Bone_cu(const Point_cu& org, const Vec3_cu& dir, float length):
        _org(org),
        _dir( dir.normalized() * length),
        _length( length )
    { }

    // -------------------------------------------------------------------------
    /// @name Getters
    // -------------------------------------------------------------------------
    IF_CUDA_DEVICE_HOST Point_cu  org()    const{ return _org;        }
    IF_CUDA_DEVICE_HOST Point_cu  end()    const{ return _org + _dir; }
    IF_CUDA_DEVICE_HOST float     length() const{ return _length;     }
    IF_CUDA_DEVICE_HOST Vec3_cu   dir()    const{ return _dir;        }

    IF_CUDA_DEVICE_HOST void set_length (float  l ){ _length = l;    }

    // -------------------------------------------------------------------------
    /// @name Setters
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST
    void set_start_end(const Point_cu& p0, const Point_cu& p1){
        _org = p0;
        _dir = p1 - p0;
        _length = _dir.norm();
    }

    IF_CUDA_DEVICE_HOST
    void set_orientation(const Point_cu& org, const Vec3_cu& dir){
        _org = org;
        _dir = dir.normalized() * _length;
    }

    // -------------------------------------------------------------------------
    /// @name Utilities
    // -------------------------------------------------------------------------

    /// 'p' is projected on the bone,
    /// then it returns the distance from the  origine '_org'
    IF_CUDA_DEVICE_HOST
    float dist_proj_to(const Point_cu& p) const
    {
        const Vec3_cu op = p - _org;
        return op.dot(_dir.normalized());
    }

    /// Orthogonal distance from the bone line to a point
    IF_CUDA_DEVICE_HOST
    float dist_ortho_to(const Point_cu& p) const
    {
        const Vec3_cu op = p - _org;
        return op.cross(_dir.normalized()).norm();
    }

    /// squared distance from a point p to the bone's segment.
    IF_CUDA_DEVICE_HOST
    float dist_sq_to(const Point_cu& p) const {
        Vec3_cu op = p - _org;
        float x = op.dot(_dir) / (_length * _length);
        x = fminf(1.f, fmaxf(0.f, x));
        Point_cu proj = _org + _dir * x;
        float d = proj.distance_squared(p);
        return d;
    }

    /// euclidean distance from a point p to the bone's segment.
    IF_CUDA_DEVICE_HOST
    float dist_to(const Point_cu& p) const {
        return sqrtf( dist_sq_to( p ) );
    }

    /// project p on the bone segment if the projection is outside the segment
    /// then returns the origine or the end point of the bone.
    IF_CUDA_DEVICE_HOST
    Point_cu project(const Point_cu& p) const
    {
        const Vec3_cu op = p - _org;
        float d = op.dot(_dir.normalized()); // projected dist from origin

        if(d < 0)            return _org;
        else if(d > _length) return _org + _dir;
        else                 return _org + _dir.normalized() * d;
    }

    /// Get the local frame of the bone. This method only guarantes to generate
    /// a frame with an x direction parallel to the bone and centered about '_org'
    IF_CUDA_DEVICE_HOST
    Transfo get_frame() const
    {
        Vec3_cu x = _dir.normalized();
        Vec3_cu ortho = x.cross(Vec3_cu(0.f, 1.f, 0.f));
        Vec3_cu z, y;
        if (ortho.norm_squared() < 1e-06f * 1e-06f)
        {
            ortho = Vec3_cu(0.f, 0.f, 1.f).cross(x);
            y = ortho.normalized();
            z = x.cross(y).normalized();
        }
        else
        {
            z = ortho.normalized();
            y = z.cross(x).normalized();
        }

        return Transfo(Mat3_cu(x, y, z), _org.to_vector() );
    }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------
// TODO: must be private
    Point_cu _org; ///< Bone origin (first joint position)
    Vec3_cu _dir;  ///< Bone direction towards its son if any
    float _length; ///< Bone length (o + v.normalized*length = bone_end_point)
};
// =============================================================================

/** @class Bone_cu
  @brief Subclass of Bone_cu.

  @see Bone_type Bone_cu
*/
#include "hermiteRBF.hpp"
#include "precomputed_prim.hpp"

struct Skeleton;

class Bone : public Bone_cu {
public:
    /// A bone identifier
    typedef int Id;

    Bone();
    ~Bone();

    Id get_bone_id() const { return _bone_id; }

    /// Get the bone type
    /// @see Bone_type
    EBone::Bone_t get_type() const {
        return !_enabled? EBone::SSD:
            _precomputed? EBone::PRECOMPUTED:
            EBone::HRBF;
    }

    /// Get the oriented bounding box associated to the bone
    OBBox_cu get_obbox() const;

    /// Get the axis aligned bounding box associated to the bone
    BBox_cu get_bbox() const;

    HermiteRBF& get_hrbf() { return _hrbf; }
    const HermiteRBF& get_hrbf() const { return _hrbf; }

    // The primitive is only populated when is_precomputed is true.  However, we'll always create
    // an empty primitive, so the primitive ID is always valid.
    Precomputed_prim& get_primitive(){ return _primitive; }
    const Precomputed_prim& get_primitive() const { return _primitive; }

    bool get_enabled() const { return _enabled; }
    void set_enabled(bool value);

    /// Set the radius of the hrbf.
    /// The radius is used to transform hrbf from global support to
    /// compact support
    void set_hrbf_radius(float rad, const Skeleton *skeleton);
    float get_hrbf_radius() const { return _hrbf.get_radius(); }

    // Precompute the HRBF, allowing get_primitive() to be called.
    void precompute(const Skeleton *skeleton);
    void discard_precompute();
    bool is_precomputed() const { return _precomputed; }

    const Skeleton &get_bone_skeleton() const { return *boneSkeleton.get(); }

    // Set the object space direction and length.  (In object space, the origin is always
    // 0,0,0.)
    //
    // When set_world_space_matrix is called, the Bone_cu base of this object is set to this
    // object space * the world space matrix.
    void set_object_space_dir(Vec3_cu b) { _object_space = b; set_world_space_matrix(_world_space_transform); }
        
    // Set the bone's coordinate system using the specified transform.
    void set_world_space_matrix(Transfo tr);
    Transfo get_world_space_matrix() const { return _world_space_transform; }

private:
    // A globally unique bone ID.
    const Id _bone_id;

    HermiteRBF _hrbf;

    bool _enabled;
    bool _precomputed;
    Precomputed_prim _primitive;
    OBBox_cu         _obbox;

    // Our orientation in world space.
    Vec3_cu _object_space;

    Transfo _world_space_transform;

    std::unique_ptr<Skeleton> boneSkeleton;
};

namespace EBone {

/// @param type Bone's type from the enum field of Bone_type namespace
std::string type_to_string(int type);

}

#endif // BONE_HPP__
