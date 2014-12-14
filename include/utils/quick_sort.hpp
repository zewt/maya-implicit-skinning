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
#ifndef QUICK_SORT_HPP__
#define QUICK_SORT_HPP__

#include <cassert>

/**
    @file quick_sort.hpp
    @brief Generic implementation of the quick sort algorithm

    Some use cases:
    @code
    // With a static array
    int tab[100];
    quick_sort(tab, 100);

    // A std::Vector
    std::vector<int> vec(100);
    quick_sort(&(vec[0]), 100);
    @endcode
*/
// =============================================================================
namespace Cpu_utils {
// =============================================================================
///////////////
// INTERFACE //
///////////////

/// Sorts the array 'data' of length 'size'
/// template T must implement the operator '<='
template<typename T>
void quick_sort(T data[], int size);

/// Sorts the arrays 'key_array' and 'val_array' of lenghts 'size'
/// according to 'key_array'. Template KEY_Tmust implement '<='
template<typename KEY_T, class VAL_T>
void quick_sort(KEY_T* key_array, VAL_T* val_array, int size);

/// Sorts the arrays 'key_array' and 'val_array' between left and right indices
/// according to 'key_array'. Template KEY_Tmust implement '<='
template<typename KEY_T, class VAL_T>
void quick_sort(KEY_T* key_array, VAL_T* val_array, int left,int right);

/// Sorts the array 'data' between the indices left and right
/// template T must implement the operator '<='
template<typename T>
void quick_sort(T data[], int left, int right);

// IMPLEMENTATION ==============================================================

template<typename T>
int partition(T data[], int left, int right, int pivot)
{
    T temp; //swap placeholder
    // switch pivot and right
    T pivot_val = data[pivot];
    data[pivot] = data[right];
    data[right] = pivot_val;
    //first run starts at index 0, used to store moved elements
    int store_index = left;

    //This is where the O(n) comes from
    for(int i = left; i < right; i++)
    {
        if (data[i] <= pivot_val)
        { //if it's less then the pivot shift it
            temp = data[i]; //swap accordingly
            data[i] = data[store_index];
            data[store_index] = temp;
            store_index++; //increment our storage location
        }
    }
    // now that we're done, get the index from our storage location
    // and move the pivot back
    // this pivot is now in its final resting place
    temp = data[store_index];
    data[store_index] = data[right];
    data[right] = temp;
    return store_index; //return our new pivot
}

// -----------------------------------------------------------------------------

template<typename T>
void quick_sort(T data[], int left, int right)
{
    int mid, new_pivot;
    if(right > left){
        mid = (left+right)/2;
        //this is the O(n) portion
        new_pivot = partition(data, left, right, mid);
        //this is the O(log n) portion
        quick_sort(data, left, new_pivot-1);
        quick_sort(data, new_pivot+1, right);
    }
    return;
}

// -----------------------------------------------------------------------------

template<typename KEY_T, class VAL_T>
int partition(KEY_T* key_array, VAL_T* val_array, int left, int right, int pivot)
{
    // switch pivot
    const KEY_T k_pivot_val = key_array[pivot];
    key_array[pivot] = key_array[right];
    key_array[right] = k_pivot_val;
    const VAL_T v_pivot_val = val_array[pivot];
    val_array[pivot] = val_array[right];
    val_array[right] = v_pivot_val;

    int store_index = left;

    for(int i = left; i < right; i++){
        if (key_array[i] <= k_pivot_val){
            // swap keys
            const KEY_T k_temp = key_array[i];
            key_array[i] = key_array[store_index];
            key_array[store_index] = k_temp;

            // swap vals
            const VAL_T v_temp = val_array[i];
            val_array[i] = val_array[store_index];
            val_array[store_index] = v_temp;

            store_index++;
        }
    }
    // Put back the pivot
    const KEY_T k_temp = key_array[store_index];
    key_array[store_index] = key_array[right];
    key_array[right] = k_temp;

    const VAL_T v_temp = val_array[store_index];
    val_array[store_index] = val_array[right];
    val_array[right] = v_temp;
    // return our new pivot :
    return store_index;
}

// -----------------------------------------------------------------------------

template<typename KEY_T, class VAL_T>
void quick_sort(KEY_T* key_array, VAL_T* val_array, int left, int right)
{
    int mid, new_pivot;
    if(right > left){
        mid = (left+right)/2;
        new_pivot = partition(key_array, val_array, left, right, mid);
        quick_sort(key_array, val_array, left, new_pivot-1);
        quick_sort(key_array, val_array, new_pivot+1, right);
    }
    return;
}

// -----------------------------------------------------------------------------

template<typename KEY_T, class VAL_T>
void quick_sort(KEY_T* key_array, VAL_T* val_array, int size)
{
    quick_sort(key_array, val_array, 0, size-1);
}

template<typename T>
void quick_sort(T data[], int size){ quick_sort(data, 0, size-1); }

}// END CUDA_UTILS_NAMESPACE ====================================================

#endif // QUICK_SORT_HPP__
