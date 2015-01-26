#ifndef MISC_UTILS_H
#define MISC_UTILS_H

#include <vector>
#include <map>
namespace MiscUtils
{
    // Given a map of child IDs to parent IDs, return a list of child IDs sorted from parent
    // to child, in depth-first order.
    //
    // For example, the input [3,-1,0,1] indicates that node 0's parent is node 3,
    // node 1 is the root node, node 2's parent is node 0, and node 3's parent is
    // node 0.
    //
    // The result of this hierarchy is [1,2,3,0].  Root nodes come first, followed
    // by their children.
    //
    // The order of sibling nodes is unspecified.  Nodes with parent indexes that
    // don't exist in the array are treated as root nodes.
    //
    // If the input contains cycles, the output will be empty and false will be returned.
    bool getHierarchyOrder(const std::map<int,int> &node_to_parent, std::vector<int> &out);
}

#endif
