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
#include "cuda_globals.hpp"

#include "camera.hpp"
#include "globals.hpp"
#include "cuda_ctrl.hpp"
#include "skeleton.hpp"
#include "conversions.hpp"

Material_cu g_ray_material;

Graph*       g_graph    = 0;
Skeleton*    g_skel     = 0;
Animesh* g_animesh = 0;

/// Return true if one of the given parameters has changed or some other scene
/// parameters since the last call of this function.
/// N.B: the first call returns always true
bool has_changed(const Camera& cam,
                 const Vec3_cu& v_plane,
                 const Point_cu& p_plane,
                 int width,
                 int height)
{
    using namespace Cuda_ctrl;
    static bool begin = true;       // First call is a changed
    static Camera prev_cam;
    static Vec3_cu  prev_v_plane;
    static Point_cu   prev_p_plane;
    static int   prev_width       = -1;
    static int   prev_height      = -1;
    static int prev_selected_joint = -1;
    static int prev_selection_size = -1;
    static Vec3_cu prev_joint_pos(0.f, 0.f, 0.f);

    // Sum every joint coordinates, not very accurate but enough to
    // detect small changes between two frame
    Vec3_cu joint_pos(0.f, 0.f, 0.f);
    if (g_skel != 0)
        for(int i=0; i<g_skel->nb_joints(); i++)
            joint_pos += g_skel->joint_pos(i);

    // extract last selected joints
    const std::vector<int>& joint_set = _skeleton.get_selection_set();
    int s = (int)joint_set.size();
    int selected_joint =  (s != 0) ? joint_set[s-1] : -1;

    // Detect differences between this frame and the last one
    float delta = 0.f;
    delta += (prev_cam.get_pos()-cam.get_pos()).norm();
    delta += prev_cam.get_dir().cross(cam.get_dir()).norm();
    delta += prev_cam.get_y().cross(cam.get_y()).norm();
    delta += fabs(prev_cam.get_frustum_width() - cam.get_frustum_width());
    delta += fabs(prev_cam.get_near() - cam.get_near());
    delta += fabs(prev_cam.get_far() - cam.get_far());
    delta += (float)(prev_cam.is_ortho() != cam.is_ortho());
    delta += (prev_p_plane-p_plane).norm();
    delta += prev_v_plane.cross(v_plane).norm();
    delta += abs(prev_width-width);
    delta += abs(prev_height-height);
    delta += (float)(prev_selected_joint != selected_joint);
    delta += (float)(prev_selection_size != s);
    delta += (joint_pos != prev_joint_pos);


    if( delta > 0.0001f || begin)
    {
        prev_cam         = cam;
        prev_v_plane     = v_plane;
        prev_p_plane     = p_plane;
        prev_width       = width;
        prev_height      = height;
        prev_selected_joint = selected_joint;
        prev_selection_size = s;
        prev_joint_pos = joint_pos;
        begin = false;

        return true;
    }

    return false;
}

// -----------------------------------------------------------------------------
