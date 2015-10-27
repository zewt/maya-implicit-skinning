#ifndef JOINT_TYPE_HPP__
#define JOINT_TYPE_HPP__

// TODO: to be moved in Skeleton_env

/** @namespace Joint_type
  @brief This namespace holds an enum field used to identify various joints types
*/
// =============================================================================
namespace EJoint{
// =============================================================================

/// @note the blending type list should be kept small. Too many blending
/// operators would slow down drastically the evaluation of the implicit skeleton
enum Joint_t {
    /// Gradient controlled operator with arc of circle profile.
    /// opening function is a diamond shape like.
    GC_ARC_CIRCLE_TWEAK = 0,
    /// clean union with the max function
    MAX,
    /// Gradient controlled bulge in contact
    BULGE,    
    NB_JOINT_T,
    NONE,
    /// Types with values higher than this enumerant
    /// will be interpreted as custom operators:
    /// Blending_env::Op_id id = enum_value - BEGIN_CUSTOM_OP_ID;
    BEGIN_CUSTOM_OP_ID
};

} // END JOINT_TYPE NAMESPACE ===================================================

#endif // JOINT_TYPE_HPP__
