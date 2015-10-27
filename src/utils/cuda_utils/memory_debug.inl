inline void cuda_print_rusage()
{
    size_t free_byte ;
    size_t total_byte ;
    cudaMemGetInfo( &free_byte, &total_byte);

    double free_db  = (double)free_byte;
    double total_db = (double)total_byte;

    free_db  = (free_db/1024.0/1024.0);
    total_db = (total_db/1024.0/1024.0);
    printf("Memory usage: \n");
    printf("Free: %f Mo\n", free_db);
    printf("Total: %f Mo\n", total_db);
    fflush(stdout);
}

// -----------------------------------------------------------------------------

inline void cuda_print_memory_trace()
{
#ifdef TRACE_MEMORY
    Memory_stack::print();
#endif
}

// -----------------------------------------------------------------------------

inline cudaError_t cuda_malloc_and_trace(void** devptr,
                                         size_t size,
                                         const char* name)
{
    cudaError_t errcode = cudaMalloc(devptr, size);
    if(errcode == cudaSuccess){
        Memory_stack::push(*devptr, size, name, Memory_stack::LINEAR_MEMORY);
    }
    return errcode;
}

// -----------------------------------------------------------------------------

inline cudaError_t cuda_malloc_array_and_trace(struct cudaArray** arrayPtr,
                                               const struct cudaChannelFormatDesc* desc,
                                               size_t width,
                                               size_t height,
                                               const char* name)
{
    cudaError_t errcode = cudaMallocArray(arrayPtr, desc, width, height);
    if(errcode == cudaSuccess){
        size_t size = ((desc->x+7)/8 + (desc->y+7)/8 + (desc->z+7)/8 + (desc->w+7)/8)*width*height;
        Memory_stack :: push(*arrayPtr, size, name, Memory_stack :: CUDA_ARRAY);
    }
    return errcode;
}

// -----------------------------------------------------------------------------

inline cudaError_t cuda_malloc_3D_and_trace(struct cudaPitchedPtr* pitchedDevPtr,
                                            struct cudaExtent extent)
{
    cudaError_t errcode = cudaMalloc3D(pitchedDevPtr, extent);
    return errcode;
}

// -----------------------------------------------------------------------------

inline cudaError_t cuda_malloc_3Darray_and_trace(struct cudaArray** arrayPtr,
                                                 const struct cudaChannelFormatDesc* desc,
                                                 struct cudaExtent extent,
                                                 const char* name)
{
#define __MAXT(x,y) ((x>y)?x:y)
    cudaError_t errcode = cudaMalloc3DArray(arrayPtr, desc, extent);
    if(errcode == cudaSuccess){
        size_t size_ch = ((desc->x+7)/8 + (desc->y+7)/8 + (desc->z+7)/8 + (desc->w+7)/8);
        size_t size = size_ch * __MAXT(extent.depth,1) * __MAXT(extent.height,1) * extent.width;
        Memory_stack::push(*arrayPtr, size, name, Memory_stack::CUDA_ARRAY);
    }
    return errcode;
#undef __MAXT
}

// -----------------------------------------------------------------------------

inline cudaError_t cuda_malloc_pitch_and_trace(void** devPtr,
                                               size_t* pitch,
                                               size_t width,
                                               size_t height,
                                               const char* name)
{
    cudaError_t errcode = cudaMallocPitch(devPtr, pitch, width, height);
    if(errcode == cudaSuccess){
        size_t size = (*pitch) * height;
        Memory_stack::push(*devPtr, size, name, Memory_stack::LINEAR_MEMORY);
    }
    return errcode;
}

// -----------------------------------------------------------------------------

inline cudaError_t cuda_free_and_untrace(void* devptr){
    cudaError_t errcode = cudaFree(devptr);
    if(errcode == cudaSuccess){
        Memory_stack::pop(devptr);
    }
    return errcode;
}

// -----------------------------------------------------------------------------

inline cudaError_t cuda_free_array_and_untrace(struct cudaArray* array){
    cudaError_t errcode = cudaFreeArray(array);
    if(errcode == cudaSuccess){
        Memory_stack::pop(array);
    }
    return errcode;
}

// -----------------------------------------------------------------------------

#ifdef TRACE_MEMORY

#define STRINGIFY(x) #x
#define STRINGIFY2(x) STRINGIFY(x)
#define cudaMallocPitch(x,y,z,w) cuda_malloc_pitch_and_trace(x,y,z,w,__FILE__ ":" STRINGIFY2(__LINE__) ":"#x)
#define cudaMalloc3DArray(x,y,z) cuda_malloc_3Darray_and_trace(x,y,z,__FILE__ ":" STRINGIFY2(__LINE__) ":"#x)
#define cudaMalloc3D(x,y) cuda_malloc_3D_and_trace(x,y)
#define cudaMallocArray(x,y,z,w) cuda_malloc_array_and_trace(x,y,z,w,__FILE__ ":" STRINGIFY2(__LINE__) ":"#x)
#define cudaMalloc(x,y) cuda_malloc_and_trace(x,y,__FILE__ ":" STRINGIFY2(__LINE__) ":"#x)
#define cudaFree(x) cuda_free_and_untrace(x)
#define cudaFreeArray(x) cuda_free_array_and_untrace(x)

#endif
