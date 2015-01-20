#include "hermiteRBF.hpp"
#include "hrbf_env.hpp"
#include "distance_field.hpp"

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

    HRBF_env::apply_hrbf_transfos();
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

IF_CUDA_DEVICE_HOST
float HermiteRBF::fngf_global(Vec3_cu& grad, const Point_cu& x) const
{
    grad = Vec3_cu(0., 0., 0.);

    float ret  = 0;
    int2 size_off = HRBF_env::fetch_inst_size_and_offset(_id);

    if(size_off.y == 0) return 0.f;

    for(int i = 0; i<size_off.y; i++)
    {
        Point_cu  node;
        Vec3_cu beta;
        float alpha     = HRBF_env::fetch_weights_point(beta, node, i+size_off.x);
        Vec3_cu diff  = x - node;

        Vec3_cu diffNormalized = diff;
        float l = diffNormalized.safe_normalize();

        // thin plates + generalisation
        #if defined(HERMITE_WITH_X3)
        float _3l      = 3 * l;
        float alpha3l  = alpha * _3l;
        float bDotd3   = beta.dot(diff) * 3;

        grad.x += alpha3l * diff.x;
        grad.x += beta.x * _3l + diffNormalized.x * bDotd3;

        grad.y += alpha3l * diff.y;
        grad.y += beta.y * _3l + diffNormalized.y * bDotd3;

        grad.z += alpha3l * diff.z;
        grad.z += beta.z * _3l + diffNormalized.z * bDotd3;

        ret += (alpha * l * l + beta.dot(diff) * 3.f) * l ;

        #elif defined(HERMITE_RBF_HPP__)
        // cf wxMaxima with function = alpha * phi(sqrt((cx-x)^2 + (cy-y)^2 + (cz-z)^2))
        //                             + dphi(sqrt((cx-x)^2 + (cy-y)^2 + (cz-z)^2)) * ((cx-x)*bx + (cy-y)*by + (cz-z)*bz) / sqrt((cx-x)^2 + (cy-y)^2 + (cz-z)^2);

        if( l > 0.00001f)
        {
            float dphi = RBFWrapper::PHI_TYPE::df(l);
            float ddphi = RBFWrapper::PHI_TYPE::ddf(l);

            float alpha_dphi = alpha * dphi;

            float bDotd_l = beta.dot(diff)/l;
            float squared_l = diff.norm_squared();

            grad.x += alpha_dphi * diffNormalized.x;
            grad.x += bDotd_l * (ddphi * diffNormalized.x - diff.x * dphi / squared_l) + beta.x * dphi / l ;

            grad.y += alpha_dphi * diffNormalized.y;
            grad.y += bDotd_l * (ddphi * diffNormalized.y - diff.y * dphi / squared_l) + beta.y * dphi / l ;

            grad.z += alpha_dphi * diffNormalized.z;
            grad.z += bDotd_l * (ddphi * diffNormalized.z - diff.z * dphi / squared_l) + beta.z * dphi / l ;

            ret += alpha * RBFWrapper::PHI_TYPE::f(l) + beta.dot(diff)*dphi/l;
        }
        #endif

    }

    return ret;
}

IF_CUDA_DEVICE_HOST
float HermiteRBF::fngf(Vec3_cu& grad, const Point_cu& x) const
{
    const float ret = fngf_global(grad, x);
#if defined(POLY_C2)
    Field::grad_to_compact_poly_c2(ret, HRBF_env::fetch_radius(_id), grad);
    return Field::to_compact_poly_c2(ret, HRBF_env::fetch_radius(_id));
#elif defined(TANH_CINF)
    Field::grad_to_compact_tanh(ret, HRBF_env::fetch_radius(_id), TO_, grad);
    return Field::to_compact_tanh(ret, HRBF_env::fetch_radius(_id), TO_);
#else
    return ret;
#endif
}

