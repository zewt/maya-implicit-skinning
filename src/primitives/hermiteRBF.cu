#include "hermiteRBF.hpp"
#include "hrbf_env.hpp"

namespace { __device__ void fix_debug() { } }

void HermiteRBF::initialize()
{
    assert(_id < 0);
    _id = HRBF_env::new_instance();
    HRBF_env::set_inst_radius(_id, 7.f);
}


void HermiteRBF::clear()
{
    assert(_id >= 0);
    HRBF_env::delete_instance(_id);
}

bool HermiteRBF::empty() const {
    assert(_id >= 0);
    return HRBF_env::get_instance_size(_id) == 0;
}

void HermiteRBF::init_coeffs(const std::vector<Vec3_cu>& nodes,
                        const std::vector<Vec3_cu>& normals)
{

    if(_id >= 0) HRBF_env::reset_instance(_id);
    else         assert(_id < 0);

    // Add nodes and compute the hrbf weights
    HRBF_env::add_samples(_id, nodes, normals);
}

/// init HRBF from samples and user defined weights
/// before calling this one must initialize the hrbf with initialize()
void HermiteRBF::init_coeffs(const std::vector<Vec3_cu>& nodes,
                        const std::vector<Vec3_cu>& normals,
                        const std::vector<float4>&  weights)
{

    if(_id >= 0) HRBF_env::reset_instance(_id);
    else         assert(_id < 0);

    // Add nodes and hrbf weights
    HRBF_env::add_samples(_id, nodes, normals, weights);
}

/// Sets the radius of the HRBF used to transform the potential field from
/// global to compact
void HermiteRBF::set_radius(float r){ HRBF_env::set_inst_radius(_id, r); }

float HermiteRBF::get_radius() const { return HRBF_env::get_inst_radius( _id ); }

void HermiteRBF::get_samples(std::vector<Vec3_cu>& list) const {
    HRBF_env::get_samples(_id, list);
}

void HermiteRBF::get_normals(std::vector<Vec3_cu>& list) const {
    HRBF_env::get_normals(_id, list);
}
