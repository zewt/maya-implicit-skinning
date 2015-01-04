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
#ifndef CUDA_MAIN_KERNELS_HPP__
#define CUDA_MAIN_KERNELS_HPP__

#include <vector>

#include "blending_env_type.hpp"

/// Initialize device memory and textures
/// @warning must be called first before any other cuda calls
void init_cuda(const std::vector<Blending_env::Op_t>& op);

#endif // CUDA_MAIN_KERNELS_HPP__
