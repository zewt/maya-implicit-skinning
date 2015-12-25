// #define USE_VCGLIB

#include "utils_sampling.hpp"
#include "bbox.hpp"
#include "idx3_cu.hpp"
#include <assert.h>
#include <functional>
#include <vector>
#include <unordered_map>
#include <map>
#include <algorithm>
using namespace std;

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#if defined(USE_VCGLIB)
#include "vcg_mesh.hpp"
#endif

/*
 * Simple poisson disk sampling.
 *
 * This is based on "Parallel Poisson Disk Sampling with Spectrum Analysis on Surfaces":
 *
 * http://research.microsoft.com/pubs/135760/c95-f95_199-a16-paperfinal-v5.pdf
 *
 * That paper targets parallel evaluation.  That's not actually implemented.  Since we don't need a lot
 * of samples, and this is only done at initialization time, it's probably not worthwhile.  However, the
 * algorithm in this paper is simple to implement and doesn't need to be paralellized.
 *
 * This works by doing a naive, dense random point sampling of the mesh to get a set of points, hashing
 * the points on a grid with a resolution of the point size that we want, then dart throwing on the points
 * to select samples.
 */
namespace 
{
float area_of_triangle(const Vec3_cu &a, const Vec3_cu &b, const Vec3_cu &c)
{
    float ab = (a - b).norm();
    float bc = (b - c).norm();
    float ca = (c - a).norm();
    float p = (ab + bc + ca) / 2;
    return sqrtf(p * (p - ab) * (p - bc) * (p - ca));
}

float random_float(float maximum)
{
    return float((double)rand()/(double)(RAND_MAX/maximum));
}

struct sample_point
{
    int tri_id;
    int cell_id;
    Point_cu pos;
    Vec3_cu normal;
};

// Estimate the geodesic distance between two points using their position and normals.
// See c95-f95_199-a16-paperfinal-v5 "3.2 Sampling with Geodesic Distance".  This estimation
// assumes that we have a smooth gradient between the two points, since it doesn't have any
// information about complex surface changes between the points.
float approximate_geodesic_distance(Point_cu p1, Point_cu p2, Vec3_cu n1, Vec3_cu n2)
{
#if 0
    return p1.distance_squared(p2);
#else
    n1.normalize();
    n2.normalize();

    Vec3_cu v = (p2-p1);
    v.normalize();

    float c1 = n1.dot(v);
    float c2 = n2.dot(v);
    float result = p1.distance_squared(p2);
    // Check for division by zero:
    if(fabs(c1 - c2) > 0.0001)
        result *= (asin(c1) - asin(c2)) / (c1 - c2);
    return result;
#endif
}

// Do a simple random sampling of the triangles.  Return the total surface area of the triangles.
float create_raw_samples(int num_samples,
    const std::vector<Vec3_cu>& verts,
    const std::vector<Vec3_cu>& nors,
    const std::vector<int>& tris,
    vector<sample_point> &samples)
{
    // Calculate the area of each triangle.  We'll use this to randomly select triangles with probability proportional
    // to their area.
    vector<float> tri_area;
    for(int tri = 0; tri < tris.size(); tri += 3)
    {
        int vert_idx_0 = tris[tri+0];
        int vert_idx_1 = tris[tri+1];
        int vert_idx_2 = tris[tri+2];
        const Vec3_cu &v0 = verts[vert_idx_0];
        const Vec3_cu &v1 = verts[vert_idx_1];
        const Vec3_cu &v2 = verts[vert_idx_2];
        float area = area_of_triangle(v0, v1, v2);
        tri_area.push_back(area);
    }

    // Map the sums of the areas to a triangle index.  For example, if we have triangles with areas
    // 2, 7, 1 and 4, create:
    //
    // 0: 0
    // 2: 1
    // 9: 2
    // 10: 3
    //
    // A random number in 0...14 can then be mapped to an index.
    map<float, int> area_sum_to_index;
    float max_area_sum = 0;
    for(int i = 0; i < tri_area.size(); ++i)
    {
        area_sum_to_index[max_area_sum] = i;
        max_area_sum += tri_area[i];
    }

    for(int i = 0; i < num_samples; ++i)
    {
        // Select a random triangle.
        float r = random_float(max_area_sum);
        auto it = area_sum_to_index.upper_bound(r);
        assert(it != area_sum_to_index.begin());
        --it;
        int tri = it->second;

        // The vertices (and corresponding normals) of the triangle that we've selected:
        int vert_idx_0 = tris[tri*3+0];
        int vert_idx_1 = tris[tri*3+1];
        int vert_idx_2 = tris[tri*3+2];
        const Vec3_cu &v0 = verts[vert_idx_0];
        const Vec3_cu &v1 = verts[vert_idx_1];
        const Vec3_cu &v2 = verts[vert_idx_2];
        const Vec3_cu &n0 = nors[vert_idx_0];
        const Vec3_cu &n1 = nors[vert_idx_1];
        const Vec3_cu &n2 = nors[vert_idx_2];

        // Select a random point on the triangle.
        float u = random_float(1), v = random_float(1);

        Vec3_cu pos = 
            v0 * (1 - sqrt(u)) +
            v1 * (sqrt(u) * (1 - v)) +
            v2 * (v * sqrt(u));

        // XXX: Is this normal calculation correct?
        Vec3_cu normal = 
            n0 * (1 - sqrt(u)) +
            n1 * (sqrt(u) * (1 - v)) +
            n2 * (v * sqrt(u));
        normal.normalize();

        samples.emplace_back();
        sample_point &p = samples.back();
        p.cell_id = -1;
        p.pos = pos.to_point();
        p.normal = normal;
        p.tri_id = tri;
    }

    return max_area_sum;
}

void poisson_disk_from_samples(float radius,
                vector<sample_point> &raw_samples,
                std::vector<Vec3_cu>& samples_pos,
                std::vector<Vec3_cu>& samples_nor)
{

    // Get the bounding box of the samples.
    BBox_cu bbox;
    for(const auto &p: raw_samples)
        bbox.add_point(p.pos);
    const Vec3_cu bbox_size = bbox.lengths();

    const float radius_squared = radius*radius;
    Vec3_cu grid_size = bbox_size / radius;
    Vec3i_cu grid_size_int(lrintf(grid_size.x), lrintf(grid_size.y), lrintf(grid_size.z));
    grid_size_int.x = max(grid_size_int.x, 1);
    grid_size_int.y = max(grid_size_int.y, 1);
    grid_size_int.z = max(grid_size_int.z, 1);

    // Assign a cell ID to each sample.
    for(auto &p: raw_samples)
    {
        Vec3i_cu idx = bbox.index_grid_cell(grid_size_int, p.pos);
        Idx3_cu offset(grid_size_int, idx);
        p.cell_id = offset.to_linear();
    }

    // Sort samples by cell ID.
    sort(raw_samples.begin(), raw_samples.end(), [](const sample_point &lhs, const sample_point &rhs) {
        return lhs.cell_id < rhs.cell_id;
    });

    struct poisson_sample {
        Point_cu pos;
        Vec3_cu normal;
    };

    struct hash_data {
        // Resulting output sample points for this cell:
        vector<poisson_sample> poisson_samples;

        // Index into raw_samples:
        int first_sample_idx;
        int sample_cnt;
    };

    // Map from cell IDs to hash_data.  Each hash_data points to the range in raw_samples corresponding to that cell.
    // (We could just store the samples in hash_data.  This implementation is an artifact of the reference paper, which
    // is optimizing for GPU acceleration that we haven't implemented currently.)
    unordered_map<int, hash_data> cells;

    {
        int last_id = -1;
        unordered_map<int, hash_data>::iterator last_id_it;

        for(int i = 0; i < raw_samples.size(); ++i)
        {
            const auto &sample = raw_samples[i];
            if(sample.cell_id == last_id)
            {
                // This sample is in the same cell as the previous, so just increase the count.  Cells are
                // always contiguous, since we've sorted raw_samples by cell ID.
                ++last_id_it->second.sample_cnt;
                continue;
            }

            // This is a new cell.
            hash_data data;
            data.first_sample_idx = i;
            data.sample_cnt = 1;

            auto result = cells.insert({sample.cell_id, data});
            last_id = sample.cell_id;
            last_id_it = result.first;
        }
    }

    // Make a list of offsets to neighboring cell indexes, and to ourself (0).
    vector<int> neighbor_cell_offsets;
    {
        Idx3_cu offset(grid_size_int, 0);
        for(int x = -1; x <= +1; ++x)
        {
            for(int y = -1; y <= +1; ++y)
            {
                for(int z = -1; z <= +1; ++z)
                {
                    offset.set_3d(x, y, z);
                    neighbor_cell_offsets.push_back(offset.to_linear());
                }
            }
        }
    }

    int max_trials = 5;
    for(int trial = 0; trial < max_trials; ++trial)
    {
        // Create sample points for each entry in cells.
        for(auto &it: cells)
        {
            int cell_id = it.first;
            hash_data &data = it.second;

            // This cell's raw sample points start at first_sample_idx.  On trial 0, try the first one.
            // On trial 1, try first_sample_idx + 1.
            int next_sample_idx = data.first_sample_idx + trial;
            if(trial >= data.sample_cnt)
            {
                // There are no more points to try for this cell.
                continue;
            }
            const auto &candidate = raw_samples[next_sample_idx];

            // See if this point conflicts with any other points in this cell, or with any points in
            // neighboring cells.  Note that it's possible to have more than one point in the same cell.
            bool conflict = false;
            for(int neighbor_offset: neighbor_cell_offsets)
            {
                int neighbor_cell_id = cell_id + neighbor_offset;
                const auto &it = cells.find(neighbor_cell_id);
                if(it == cells.end())
                    continue;

                const hash_data &neighbor = it->second;
                for(const auto &sample: neighbor.poisson_samples)
                {
                    float distance = approximate_geodesic_distance(sample.pos, candidate.pos, sample.normal, candidate.normal);
                    if(distance < radius_squared)
                    {
                        // The candidate is too close to this existing sample.
                        conflict = true;
                        break;
                    }
                }
                if(conflict)
                    break;
            }

            if(conflict)
                continue;

            // Store the new sample.
            data.poisson_samples.emplace_back();
            poisson_sample &new_sample = data.poisson_samples.back();
            new_sample.pos = candidate.pos;
            new_sample.normal = candidate.normal;
        }
    }

    // Copy the results to the output.
    for(const auto it: cells)
    {
        for(const auto &sample: it.second.poisson_samples)
        {
            samples_pos.push_back(sample.pos.to_vector());
            samples_nor.push_back(sample.normal);

            // printf("-> %.1f %.1f %.1f\n", sample.pos.x, sample.pos.y, sample.pos.z);
        }
    }
}
}

namespace Utils_sampling {

void poisson_disk(float radius,
                  int nb_samples,
                  const std::vector<Vec3_cu>& verts,
                  const std::vector<Vec3_cu>& nors,
                  const std::vector<int>& tris,
                  std::vector<Vec3_cu>& samples_pos,
                  std::vector<Vec3_cu>& samples_nor)
{
    assert(verts.size() == nors.size());
    assert(verts.size() > 0);
    assert(tris.size() > 0);

#if defined(USE_VCGLIB)
    vcg::MyMesh vcg_mesh, sampler;
    vcg_mesh.concat((float*)&(verts[0]), (int*)&(tris[0]), verts.size(), tris.size()/3);
    vcg_mesh.set_normals( (float*)&(nors[0]) );
    vcg_mesh.update_bb();

    vcg::MyAlgorithms::Poison_setup pp;
    pp._radius = radius;
    pp._nb_samples = nb_samples;
    pp._approx_geodesic_dist = true;
    vcg::MyAlgorithms::poison_disk_sampling(vcg_mesh, pp, sampler);

    const int nb_vert = sampler.vert.size();
    samples_pos.clear();
    samples_nor.clear();
    samples_pos.resize( nb_vert );
    samples_nor.resize( nb_vert );
    vcg::MyMesh::VertexIterator vi = sampler.vert.begin();
    for(int i = 0; i < nb_vert; ++i, ++vi)
    {
        vcg::MyMesh::CoordType  p = (*vi).P();
        vcg::MyMesh::NormalType n = (*vi).N();
        samples_pos[i] = Vec3_cu(p.X(), p.Y(), p.Z());
        samples_nor[i] = Vec3_cu(n.X(), n.Y(), n.Z());
    }

    // sampler.get_vertices( samples_pos );
#else
    // Create random sample points to sample.  We usually aren't doing very high-resolution sampling,
    // so we don't need a lot.
    vector<sample_point> raw_samples;
    float surface_area = create_raw_samples(10000, verts, nors, tris, raw_samples);

    if(radius <= 0)
    {
        // Estimate the radius from the number of samples and the surface area of the mesh.
        // This estimation pretends that the surface is a cube, and the samples are spaced
        // evenly across it.  0.75 is an empirical adjustment.
        radius = sqrtf(surface_area / nb_samples) * 0.75f;
    }

    // XXX: This is pretty fast.  Since our radius is an estimate and we can end up with too many
    // or too few samples, should we do a search to find the radius resulting in a closer number of
    // samples to what was requested?
    poisson_disk_from_samples(radius, raw_samples, samples_pos, samples_nor);
#endif
}

}// END UTILS_SAMPLING NAMESPACE ===============================================
