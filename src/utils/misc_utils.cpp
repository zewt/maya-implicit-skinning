#include "misc_utils.hpp"
#include <map>
#include <set>
using namespace std;

static bool addHierarchyOrderRecursive(const map<int, vector<int> > children, vector<int> &out, int rootIdx, set<int> &indexesOnStack, set<int> &reachedIndexes)
{
    // If this index is already on the stack, then the input has a cycle.
    if(indexesOnStack.find(rootIdx) != indexesOnStack.end())
        return false;

    indexesOnStack.insert(rootIdx);
    reachedIndexes.insert(rootIdx);

    // Process this node before its children.
    out.push_back(rootIdx);

    // Recurse through all children of this node.
    bool result = true;
    if(children.find(rootIdx) != children.end()) {
        const vector<int> &childIndexes = children.at(rootIdx);
        for(int childIdx: childIndexes)
        {
            if(!addHierarchyOrderRecursive(children, out, childIdx, indexesOnStack, reachedIndexes))
                result = false;
        }
    }

    indexesOnStack.erase(rootIdx);
    return true;
}

bool MiscUtils::getHierarchyOrder(const std::vector<int> &parent, std::vector<int> &out)
{
    // Make a list of each node's children.
    map<int, vector<int> > children;
    for(int idx = 0; idx < (int) parent.size(); ++idx) {
        int parent_idx = parent[idx];
        if(parent_idx >= 0 && parent_idx < parent.size())
            children[parent_idx].push_back(idx);
    }

    // Start processing root nodes.
    set<int> indexesOnStack;
    set<int> reachedIndexes;
    for(int idx = 0; idx < (int) parent.size(); ++idx)
    {
        int parent_idx = parent[idx];
        if(parent_idx >= 0 && parent_idx < parent.size())
            continue;

        if(!addHierarchyOrderRecursive(children, out, idx, indexesOnStack, reachedIndexes))
        {
            out.clear();
            return false;
        }
    }

    // If we don't reach all nodes, then we either have a cycle or an out-of-bounds index.
    if(reachedIndexes.size() != parent.size())
        return false;

    return true;
}
