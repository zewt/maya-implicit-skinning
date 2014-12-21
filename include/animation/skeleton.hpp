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
#ifndef SKELETON_HPP__
#define SKELETON_HPP__

#include <vector>

#include "dual_quat_cu.hpp"
#include "bone.hpp"
#include "joint_type.hpp"
#include "transfo.hpp"
#include "cuda_utils.hpp"
#include "kinematic.hpp"
#include "blending_lib/controller.hpp"
#include "skeleton_env_type.hpp"

// Forward definitions ---------------------------------------------------------
struct Graph;
namespace Loader {
struct Abs_skeleton;
}
// End Forward definitions  ----------------------------------------------------

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// -----------------------------------------------------------------------------

/**
 *  @class Skeleton
    @brief The class that defines an articulated skeleton.
    Skeleton is represented by a tree of bones with a single joint as root
    and others as node and leaves

    We refer to the rest position or bind pose of the skeleton
    to define the initial position (i.e the moment when the skeleton/graph is
    attached to the mesh). Transformed position of the skeleton and mesh is
    the animated position computed according to the rest/bind position.

    Each joint defines the next bone except for the leaves which are bones with
    a length equal to zero :
    @code

    joint_0  bone_0   joint_1   bone_1   joint_3
    +------------------+------------------+
                     joint_2
                        \
                         \ bone_2
                          \
                           + joint_4

    @endcode
    In this example we can see that joint_1 and joint_2 has the same position
    but not the same orientation. joint_3 and joint_4 are the leaves of the
    skeleton tree the corresponding  bone length cannot be defined.
    Note that a joint frame (fetched with 'joint_anim_frame()' for instance) is
    usually different from the bone frame ( bone_anim_frame() )

    One way to look up the skeleton's bones is to explore the bone array :
    @code
    Skeleton skel;
    ...
    for(int i = 0; i < skel.get_nb_joints(); i++){

        if( skel.is_leaf(i) )
        {
            // Do something or skip it
        }

        // Acces the ith bone :
        skel.get_bone(i)->a_bone_method();
        const Bone* b = skel.get_bones(i);

        ...
    }
    @endcode

    Animating the skeleton is done through the class interface Kinematic.
    Hence '_kinec' attribute of the skeleton provides means to move bones
 */
struct Skeleton {

  /// Build skeleton from a graph
  Skeleton(const Graph& g, int root = 0);

  /// Build skeleton from the abstract representation
  Skeleton(const Loader::Abs_skeleton& skel);

  ~Skeleton();

  /// Reset the position of the skeleton to the resting position and save its
  /// position
  void reset();

  /// Restore the skeleton position  since the LAST call of reset()
  /// @see reset()
  void unreset();

  /** At each joints we compute the half angle vector formed by the adjacents
      bones. When a joint is connected to more than two bones we compute the
      average half angle. When bones are colinear the half angle is set to null.

      @code
      half angle
         ^
         |
       + | +
    b0  \|/  b1
         +
      @endcode

      We also compute the vector orthogonal to the adjacents bones. Again when
      their is more than two bones we average this orthogonal vector

      @param half_angles the list of half angles which size equals
      get_nb_joints()
      @param orthos the list of orthogonals vectors which size equals
      get_nb_joints()
  */
  void compute_joints_half_angles(Cuda_utils::HA_Vec3_cu& half_angles,
                                  Cuda_utils::HA_Vec3_cu& orthos);

  /// get skeleton hierachy with bone types in string
  std::string to_string();

  //----------------------------------------------------------------------------
  /// @name IO
  //----------------------------------------------------------------------------

  /// Save user transfo
  void save_pose(const std::string& filepath);

  /// Load user transfo
  void load_pose(const std::string& filepath);

  //----------------------------------------------------------------------------
  /// @name Setters
  //----------------------------------------------------------------------------

  /// Set the radius of the hrbf of bone i.
  /// The radius is used to transform hrbf from global support to
  /// compact support
  void set_bone_hrbf_radius(int i, float radius);

  /// Set a new resting position 'pt' for 'joint_id'
  void set_joint_rest_pos(int joint_id, const Point_cu& pt);

  /// Scale and offset the graph
  void set_offset_scale(const Vec3_cu& offset, float scale);

  void set_joint_controller(Blending_env::Ctrl_id i,
                            const IBL::Ctrl_setup& shape);

  /// @param type The joint type in the enum field in EJoint namespace
  void set_joint_blending(int i, EJoint::Joint_t type);

  /// @param m magnitude for the ith joint. range: [0 1]
  void set_joint_bulge_mag(int i, float m);

  /// Replace and delete the ith bone with the user allocated bone 'b'
  void set_bone(int i, Bone* b);

  /// Set the implicit cylinder radius of bone i
  void set_bone_radius(int i, float radius);

  //----------------------------------------------------------------------------
  /// @name Getter
  /// The difference between a joint and a bone must be clear in this section.
  /// A joint is between two bones except for the root joint. The joint frame
  /// used to compute skinning can be different from the bone frame.
  //----------------------------------------------------------------------------

  /// Get the "root" joint, i.e. the highest joint in the hierarchy
  int root() const { return _root; }

  /// Get the number of joints in the skeleton
  int nb_joints() const { return _nb_joints; }

  /// Get the position of a joint in animated position
  Vec3_cu joint_pos(int joint) const;

  /// Get the position of a joint in rest position
  Vec3_cu joint_rest_pos(int joint);

  /// Get the frame of the joint in animated position
  Transfo joint_anim_frame(int joint) const { return _anim_frames[joint];  }

  /// Get the frame of the joint in rest position
  Transfo joint_frame(int joint) const { return _frames[joint];  }

  /// Get the inverse frame of the joint in rest position
  Transfo joint_frame_lcl(int joint) const { return _lcl_frames[joint];  }

  /// Get the frame of the bone in animated position
  Transfo bone_anim_frame(int bone) const { return _h_transfos[bone] * _bones[bone].get_frame(); }

  /// Get the frame of the bone in rest position
  Transfo bone_frame(int bone) const { return _bones[bone].get_frame(); }

  IBL::Ctrl_setup get_joint_controller(int i);

  /// Get the list of children for the ith bone
  const std::vector<int>& get_sons(int i) const { return _children[i];  }

  /// Get the array that contains data on the children of each joint
  const std::vector< std::vector<int> >& get_joints_sons() const { return _children; }

  int parent(int i) const { return _parents[i]; }

  bool is_leaf(int i) const { return _children[i].size() == 0; }

  /// Get the animated bones of the skeleton
  const std::vector<Bone*>& get_bones() const { return _anim_bones; }

  /// @note A bone is a part of the skeleton, as such you cannot change its
  /// properties outside the skeleton class. Changes are to be made with the
  /// dedicated setter 'set_bone()' by creating a new bone. The setter will
  /// ensure that the skeleton updates its state according to the bone properties.
  /// Do not even think about using the dreadfull const_cast<>(). <b> You've been
  /// warned. <\b>
  const Bone* get_bone(Bone::Id i) const{ return _anim_bones[i];  }

  /// Bone in rest position
  Bone_cu get_bone_rest_pose(int i) const{ return _bones[i];  }

  Blending_env::Ctrl_id get_ctrl(int joint) const {
      return _joints_data[joint]._ctrl_id;
  }

  float get_joints_bulge_magnitude(Bone::Id i) const {
      return _joints_data[i]._bulge_strength;
  }

  Skeleton_env::DBone_id get_bone_didx(Bone::Id i) const;

  /// Number of skeleton's bone of type 'type'
  int get_nb_bone_of_type(EBone::Bone_t type);

  /// bone type (wether a primitive is attached to it)
  /// @see Bone Bone_hrbf Bone_ssd Bone_cylinder Bone_precomputed EBone
  EBone::Bone_t bone_type(Bone::Id id_bone) const {
      return _anim_bones[id_bone]->get_type();
  }

  /// @return The joint type in the enum field in EJoint namespace
  EJoint::Joint_t joint_blending(Bone::Id i) const {
      return _joints_data[i]._blend_type;
  }

  /// Get the animated joints global transformations expressed with matrices.
  /// These transformation can be used as is to deformed the mesh
  const Transfo& get_transfo(Bone::Id bone_id) const;

  /// Get the animated joints global transformations expressed with dual
  /// quaternions. These transformation can be used as is to deformed the mesh
  const Dual_quat_cu& get_dual_quat(Bone::Id bone_id) const;

  /// Get the animated joints global transformations expressed with matrices
  /// in device memory. These transformation can be used as is to deformed
  /// the mesh
  const Transfo* d_transfos() const { return _d_transfos.ptr(); }

  /// Get the animated joints global transformations expressed with dual
  /// quaternions in device memory. These transformation can be used as is
  /// to deformed the mesh
  const Dual_quat_cu* d_dual_quat() const { return _d_dual_quat.ptr(); }

  /// @return the hrbf id associated to the bone or -1
  /// if the bone is not an HRBF
  int get_hrbf_id(Bone::Id bone_id) const;

  float get_hrbf_radius(Bone::Id bone_id);

  /// Skeleton kinematic handler.
  /// Animate the skeleton through this interface
  /// @note this helps to seperating code between pure skeleton data
  /// and kinematic algorithms. Might even enable in the future to use
  /// class polymorphism in order to specialize kinematic behavior.
  Kinematic* _kinec;

  //----------------------------------------------------------------------------
  /// @name Class tools
  //----------------------------------------------------------------------------

private:

  typedef Cuda_utils::Host::PL_Array<Transfo> HPLA_tr;

  /// Factorization of every attributes allocations and inits
  void init(int nb_joints);

  /// Create and initilize a skeleton in the environment Skeleton_env
  void init_skel_env();

  friend class Kinematic; // allowskinematic to access update_anim_pose()

  /// Compute the global trannsformations '_h_transfos' with the current
  /// kinematic pose. Then updates position of the skeleton's bones
  void update_anim_pose();

  /// Given a set of global transformation at each joints animate the skeleton.
  /// animated bones frames dual quaternions are updated as well as device
  /// memory
  void update_bones_pose(const HPLA_tr& global_transfos);

  /// transform implicit surfaces computed with HRBF.
  /// @param global_transfos array of global transformations for each bone
  /// (device memory)
  void transform_hrbf(const Cuda_utils::Device::Array<Transfo>& d_global_transfos);

  /// transform implicit surfaces pre computed in 3D grids
  /// @param global_transfos array of global transformations for each bone
  void transform_precomputed_prim(const HPLA_tr& global_transfos);

  /// updates 'hrbf_id_to_bone_id' attributes according to the bone array
  // TODO: to be deleted
//  void update_hrbf_id_to_bone_id();

  /// Tool function for update_vertices() method. This updates '_anim_bones'
  /// and '_anim_frames'
  void subupdate_vertices(int root,
                          const HPLA_tr& global_transfos);

  /// Given a graph 'g' build the corresponding skeleton with the node root as
  /// root of the tree. This fills the attributes '_parents' '_children' and
  /// '_bone_lengths'
  void fill_children(Graph& g, int root);

  /// Once 'fill_children()' has been called this compute
  /// the frames at each bone. Attributes '_frames' and '_lcl_frames' are
  /// filled
  void fill_frames(const Graph& g);

  /// Once '_frames' and '_children' are filled (manually or with
  /// fill_children() and fill_frame() ) this compute '_bones' and
  /// '_anim_bones' according to the frames positions.
  void fill_bones();

  void rec_to_string(int id, int depth, std::string& str);

  //----------------------------------------------------------------------------
  /// @name Attributes
  //----------------------------------------------------------------------------

  /// Id of the skeleton in the skeleton environment
  Skeleton_env::Skel_id _skel_id;

  /// List of children IDs for the bone of identifier boneID :
  /// children[bone_ID][List_of_childIDs]
  std::vector< std::vector<Bone::Id> > _children;
  /// Map each bone to its parent : parents[bone_ID] = parent_bone_ID
  std::vector<Bone::Id> _parents;

  /// Joints frames in rest position
  std::vector<Transfo> _frames;

  /// Inverse of the Joints frame in rest position
  std::vector<Transfo> _lcl_frames;

  /// last state of '_h_transfos' corresponding of the last call of 'reset()'
  std::vector<Transfo> _saved_transfos;

  /// Joints frame animated
  std::vector<Transfo> _anim_frames;

  int _nb_joints;
  /// Id root joint
  Bone::Id _root;

  /// Skeleton offset
  Vec3_cu _offset;

  /// Skeleton scale
  float _scale;

  /// List of transformations associated to each bone in order to deform a mesh.
  /// A point will follow rigidly the ith bone movements if it is transformed
  /// by bone_transformations[parents[ith]].
  Cuda_utils::Host::PL_Array<Transfo> _h_transfos;
  /// same as h_transform but in device memory
  Cuda_utils::Device::Array<Transfo> _d_transfos;
  /// same as h_transform but represented with dual quaternions instead of matrices
  Cuda_utils::Host::PL_Array<Dual_quat_cu> _h_dual_quat;
  /// Same as h_dq_transformations but in device memory
  Cuda_utils::Device::Array<Dual_quat_cu> _d_dual_quat;

  /// Array of each animated bone. Note that a bone can be subclass.
  /// @see bone.hpp
  std::vector<Bone*> _anim_bones;

  /// Array of bones in rest position.
  /// @warning a bone can have a orientation different from its associated joint
  /// Bones orientations are solely deduce from joint positions. _frame[i]
  /// is not necessarily equal to the _bones[i] frame.
  /// @see bone.hpp
  std::vector<Bone_cu> _bones;

  // TODO: this list might not be really needed as blending env already stores it
  /// shape of the controller associated to each joint
  /// for the gradient blending operators
  Cuda_utils::Host::Array<IBL::Ctrl_setup> _controllers;

  std::vector<Skeleton_env::Joint_data> _joints_data;

  /// hrbf_id_to_bone_id[hrbf_id] = bone_id
  // TODO: to be deleted
  //std::vector<int> _hrbf_id_to_bone_id;

  /// hrbf radius to go from global to compact support
  std::vector<float> _hrbf_radius;
};

#endif // SKELETON_HPP__
