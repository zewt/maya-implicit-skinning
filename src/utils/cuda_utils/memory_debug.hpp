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
#ifndef MEMORY_DEBUG_HPP__
#define MEMORY_DEBUG_HPP__


#include <stdio.h>
#define DEFAULT_STACK_SIZE 32
#define MAX_NAME_LEN 64

/** @brief Memory toolkit designe to keep CUDA memory traces

    When TRACE_MEMORY is define every call to cudaMallocXXX() and cudaFreeXXX()
    is replaced with a similar function which perform the call and register
    it. This toolkit give means to register those calls wherever they are
    with static methods.

    A call to cudaMallocXXX() is registered by pushing it into a stack
    and call to cudaFreeXXX() pops it. The stack can be printed whenever
    needed with Memory_stack::print().

    N.B: If TRACE_MEMORY is defined convenient macro CUDA_SAFE_CALL(x) defined
    in cuda_utils.hpp prints the memory stack when a CUDA error occurs.

    @see cuda_utils.hpp CUDA_SAFE_CALL(x)
*/
struct Memory_stack{

    typedef enum{
        LINEAR_MEMORY,
        CUDA_ARRAY
    } mem_kind;

    static void push(const void* address, size_t size, const char* name, mem_kind type);

    static void pop(const void* address);

    static void print();



private:
    struct mem_s{
        inline mem_s() {}
        inline mem_s(const void* a,
                     size_t s,
                     const char* n,
                     mem_kind k):
            address(a),
            size(s),
            kind(k)
        {
            for(int i = 0; i < MAX_NAME_LEN-1; i++){
                name[i] = n[i];
                if(n[i] == '\0')
                    break;
            }
            name[MAX_NAME_LEN-1] = '\0';
        }

        const void* address;
        size_t size;
        mem_kind kind;
        char name[MAX_NAME_LEN];

    };

    static mem_s* entries;
    static int stack_size;
    static int n;

    static void realloc();
};

#endif // MEMORY_DEBUG_HPP__
