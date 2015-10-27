#ifndef TREE_HPP
#define TREE_HPP

#include <vector>
#include <map>
#include <set>
#include "bone.hpp"
#include "tree_type.hpp"

// =============================================================================
namespace Skeleton_env {
// =============================================================================

/// CPU representation of the tree
struct Tree {
    Tree(const std::vector<const Bone*>& bones,
        const std::map<Bone::Id, Bone::Id>& parents);

    /// Compute the axis aligned bounding box of the tree
    BBox_cu bbox() const;

    // -------------------------------------------------------------------------
    /// @name Getter and Setters
    // -------------------------------------------------------------------------

    void set_bones(const std::vector<const Bone*>& bones) {
        for(const Bone *bone: bones)
            _bones.at(bone->get_bone_id()) = bone;
    }

    void set_joints_data(const std::map<Bone::Id, Joint_data>& datas){ _datas = datas; }

    bool is_leaf(Bone::Id bid) const { return _sons.at(bid).size() == 0; }

          std::vector<Bone::Id>& sons(Bone::Id hid)       { return _sons.at(hid); }
    const std::vector<Bone::Id>& sons(Bone::Id hid) const { return _sons.at(hid); }

    Joint_data& data(Bone::Id hid){ return _datas.at(hid); }
    const Joint_data& data(Bone::Id hid) const { return _datas.at(hid); }

    const Bone* bone(Bone::Id hid) const { return _bones.at(hid); }
    
    const std::set<const Bone*> bones() const {
        std::set<const Bone*> result;
        for(auto &it: _bones)
            result.insert(it.second);
        return result;
    }

    // int bone_size() const { return (int)_bones.size(); }

    Bone::Id parent(Bone::Id hid) const { return _parents.at(hid); }

    // -------------------------------------------------------------------------
    /// @name Attributes
    // -------------------------------------------------------------------------

private:
    std::map<Bone::Id, std::vector<Bone::Id> > _sons;
    std::map<Bone::Id, Joint_data> _datas;
    std::map<Bone::Id, const Bone*> _bones;
    std::map<Bone::Id, Bone::Id> _parents;
};


}// NAMESPACE END Skeleton_env  ================================================

#endif // TREE_HPP
