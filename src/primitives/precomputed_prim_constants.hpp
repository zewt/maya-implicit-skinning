#ifndef PRECOMPUTED_PRIM_CONSTANTS_HPP__
#define PRECOMPUTED_PRIM_CONSTANTS_HPP__

/// With GPU of compute capability 2.0 a 3D texture maximum size is 2048^3
/// change this to match your requirements
#define MAX_TEX_LENGTH 2048

/// Grid resolution to precompute an implicit primitive in the x, y and z axis
/// (nb grid elements == GRID_RES^3)
/// @warning 128 is the higher you can choose. Unless you want to blow up memory
#define GRID_RES 64 //32

#define GRID_RES_3 (GRID_RES*GRID_RES*GRID_RES)

#endif // PRECOMPUTED_PRIM_CONSTANTS_HPP__
