#include "grid3_cu.hpp"


// -----------------------------------------------------------------------------

template<class T>
Grid3_cu<T>::

Grid3_cu(const std::vector< Grid3_cu<T>* >& list,
         const Vec3i_cu& max_s,
         std::vector<Idx3_cu> &out_idx)
{
    assert( list.size() );
    Vec3i_cu s = list[0]->_size;

    #ifndef NDEBUG
    assert( s.x <= max_s.x && s.y <= max_s.y && s.z <= max_s.z );
    // Every grid in list should have same size
    for (unsigned i = 1; i < list.size(); ++i) assert( list[i]->_size == s);
    #endif

    // compute the new grid dimensions
    unsigned int nb_max_x = max_s.x / s.x;
    unsigned int nb_max_y = max_s.y / s.y;
    unsigned int dim_x, dim_y, dim_z;
    if (nb_max_x > list.size()){
        dim_x = list.size();
        dim_y = 1;
        dim_z = 1;
    } else if (nb_max_x * nb_max_y > list.size()){
        dim_x = nb_max_x;
        dim_y = list.size() / nb_max_x + 1;
        dim_z = 1;
    } else {
        dim_x = nb_max_x;
        dim_y = nb_max_y;
        dim_z = list.size() / (nb_max_x*nb_max_y) + 1;
    }

    // concatenate input grids and compute out_idxs
    out_idx.clear();
    _size = Vec3i_cu(dim_x * s.x, dim_y * s.y, dim_z * s.z);
    _vals.resize( _size.product() );
    Idx3_cu id(_size, 0);
    for(unsigned i = 0; i < list.size(); ++i)
    {
        Vec3i_cu v = id.to_3d();
        out_idx.push_back( Idx3_cu(_size, v + list[i]->_pad_off) );

        for(Idx3_cu idx(s, 0); idx.is_in(); ++idx){
            _vals[ (id+idx.to_3d()).to_linear() ] = list[i]->_vals[idx.to_linear()];
        }
        // put id to end of list[i] grid in _vals dims
        if (v.x < _size.x - s.x)
            id = id + Idx3_cu(s, s.x-1, 0, 0).to_3d();
        else if (v.x < _size.x - s.x)
            id = id + Idx3_cu(s, s.x-1, s.y-1, 0).to_3d();
        else
            id = id + Idx3_cu(s, s.x-1, s.y-1, s.z-1).to_3d();
        // increment id to ensure it points to the begin of list[i+1]
        ++id;
    }
    _pad_off = Vec3i_cu::zero();
}

// -----------------------------------------------------------------------------

template <class T>
void Grid3_cu<T>::padd(const Vec3i_cu& padding, Pad_t type, T val)
{
    Vec3i_cu s = _size + padding;

    assert( _size.x < s.x && _size.y < s.y && _size.z < s.z );

    // get offsets for padding;
    Vec3i_cu off = (s - _size) / 2;
    Idx3_cu offset(s, off);
    // do custom padding / init p_v
    std::vector<T> p_v(s.x * s.y * s.z, val);
    // copy vals
    for (Idx3_cu id(_size, 0); id.is_in(); id++ ){
        int i = (id.to_3d() + offset).to_linear();
        p_v[ i ] = _vals[ id.to_linear() ];
    }
    // do copy padding
    if (type == COPY)
    {
        int dx = s.x - off.x;
        int dy = s.y - off.y;
        int dz = s.z - off.z;
        // incremental faces extrusion : z-axis, y-axis, x-axis
        for (Idx3_cu id(Vec3i_cu(_size.x, _size.y, off.z), 0); id.is_in(); ++id){
            Vec3i_cu vid = id.to_3d();
            p_v [ (vid + Idx3_cu(s, off.x, off.y,  0)).to_linear() ] = p_v[ Idx3_cu(s, off.x + vid.x, off.y + vid.y, off.z).to_linear() ];
            p_v [ (vid + Idx3_cu(s, off.x, off.y, dz)).to_linear() ] = p_v[ Idx3_cu(s, off.x + vid.x, off.y + vid.y,  dz-1).to_linear() ];
        }
        for (Idx3_cu id(Vec3i_cu(_size.x, off.y, s.z), 0); id.is_in(); ++id){
            Vec3i_cu vid = id.to_3d();
            p_v [ (vid + Idx3_cu(s, off.x,  0, 0)).to_linear() ] = p_v[ Idx3_cu(s, off.x + vid.x, off.y , vid.z).to_linear() ];
            p_v [ (vid + Idx3_cu(s, off.x, dy, 0)).to_linear() ] = p_v[ Idx3_cu(s, off.x + vid.x, dy - 1, vid.z).to_linear() ];
        }
        for (Idx3_cu id(Vec3i_cu(off.x, s.y, s.z), 0); id.is_in(); ++id){
            Vec3i_cu vid = id.to_3d();
            p_v [ (vid + Idx3_cu(s,  0, 0, 0)).to_linear() ] = p_v[ Idx3_cu(s, off.x, vid.y, vid.z).to_linear() ];
            p_v [ (vid + Idx3_cu(s, dx, 0, 0)).to_linear() ] = p_v[ Idx3_cu(s,  dx-1, vid.y, vid.z).to_linear() ];
        }
    }
    // update this
    _vals.clear();
    _vals.swap( p_v );
    _size = s;
    _pad_off += off;
}

// -----------------------------------------------------------------------------

template <class T>
cudaArray* Grid3_cu<T>::

to_gpu() const
{
    cudaArray* d_ptr = 0;
    cudaChannelFormatDesc cfd = cudaCreateChannelDesc<T>();
    cudaExtent array_size = make_cudaExtent(_size.x, _size.y, _size.z);
    CUDA_SAFE_CALL(cudaMalloc3DArray(&d_ptr, &cfd, array_size) );

    // copy data to 3D array
    cudaMemcpy3DParms copyParams = {0};
    copyParams.srcPtr   = make_cudaPitchedPtr((void*)_vals.data(),
                                              array_size.width*sizeof(T),
                                              array_size.width,
                                              array_size.height);
    copyParams.dstArray = d_ptr;
    copyParams.extent   = array_size;
    copyParams.kind     = cudaMemcpyHostToDevice;
    CUDA_SAFE_CALL( cudaMemcpy3D(&copyParams) );
    return d_ptr;
}

// -----------------------------------------------------------------------------

template <class T>
void Grid3_cu<T>::

init_vals(const T *vals)
{
    int nb_elt = _size.x * _size.y * _size.z;
    _vals.resize( nb_elt );
    for (int i = 0; i < nb_elt; ++i) {
        _vals[i] = vals[i];
    }
}

// -----------------------------------------------------------------------------
