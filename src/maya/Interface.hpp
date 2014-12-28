#ifndef PLUGIN_INTERFACE_H
#define PLUGIN_INTERFACE_H

#include "loader_mesh.hpp"
#include "loader_skel.hpp"

#include <vector>
using namespace std;

struct PluginInterfaceImpl;
class PluginInterface
{
public:
    PluginInterface();
    ~PluginInterface();

    static void init();
    static void shutdown();

    bool is_setup() const;
    void setup(const Loader::Abs_mesh &loader_mesh, const Loader::Abs_skeleton &loader_skeleton);

    void update_skeleton(const vector<Loader::CpuTransfo> &bone_positions);
    void update_vertices(const vector<Loader::Vertex> &loader_vertices);
    void go(vector<Loader::Vec3> &out_verts);

private:
    PluginInterfaceImpl *impl;
};
#endif
