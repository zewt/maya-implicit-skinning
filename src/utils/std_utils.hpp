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
#ifndef STD_UTILS_HPP__
#define STD_UTILS_HPP__

#include <cassert>
#include <vector>
#include <map>
#include <sstream>
#include <algorithm>

/**
    @namespace Std_utils
    @brief Utilities for the stl std::vector std::map etc.

    usage:
    @code



    @endcode

    Signature nomenclature :
    bool exists(std::container, ELT)
    bool find(std::container, ELT, RES&)
    RES& find(std::container, ELT)
*/
// =============================================================================
namespace Std_utils {
// =============================================================================

/// Convert a scalar (int float double long unsigned etc.) to a string
/// @warning no type checking is done
// Possibly later we could specialized the template
template<typename T>
static std::string to_string(T number)
{
   std::stringstream ss;
   ss << number;
   return ss.str();
}

/// Pops the ith element by swapping the last element of the vector with it
/// and decrementing the size of the vector
template <class T>
static void pop(std::vector<T>& vec, int i)
{
    assert(vec.size() > 0);
    vec[i] = vec[vec.size() - 1];
    vec.pop_back();
}

// -----------------------------------------------------------------------------

/// @return true if v0 and v1 are equal in size and their elements match
template<class T0, class T1>
static bool equal(std::vector<T0>& v0, std::vector<T1>& v1)
{
    if(v0.size() != v1.size())
        return false;

    for(unsigned i = 0; i < v1.size(); ++i)
        if(v0[i] != v1[i]) return false;

    return true;
}

// -----------------------------------------------------------------------------

/// Copy src into dst (dst is resized ). their types can be different as long as
/// T0 and T1 are equal in terms of byte size.
template<class T0, class T1>
static void copy(std::vector<T0>& dst, const std::vector<T1>& src)
{
    assert(sizeof(T0) == sizeof(T1));
    dst.resize( src.size() );
    for(unsigned i = 0; i < src.size(); i++)
        dst[i] = *( reinterpret_cast<const T0*>( &(src[i]) ) );
}

// -----------------------------------------------------------------------------

/// Find 'elt' in 'vec'. Search is O(n).
template<class T0, class T1>
static bool exists(const std::vector<T0>& vec, const T1& elt)
{
    for(unsigned i = 0; i < vec.size(); i++)
        if(vec[i] == elt) return true;

    return false;
}


// Read a value from a map, returning a default value if there's no entry.
// http://stackoverflow.com/questions/2333728
template <template<class,class,class...> class C, typename K, typename V, typename... Args>
V get(const C<K,V,Args...>& m, K const& key, const V & defval)
{
    typename C<K,V,Args...>::const_iterator it = m.find( key );
    if (it == m.end())
        return defval;
    return it->second;
}

// -----------------------------------------------------------------------------

/// Find and retreive an element from the map. If not found an assertion is
/// triggered
/// @param elt : the element to be found
/// @return what is associated with 'k' in 'map'.
// third template parameter is here to avoid ambiguities
template<class Key, class Elt, class PKey>
static const Elt& find(const std::map<Key, Elt>& map, const PKey& k)
{
    typename std::map<Key, Elt>::const_iterator it = map.find( k );
    if( it != map.end() )
        return it->second;
    else
    {
        assert( false );
        return map.begin()->second;
    }
}

// -----------------------------------------------------------------------------

/// Find and retreive an element from the map. If not found an assertion is
/// triggered
/// @param elt : the element to be found
/// @return what is associated with 'k' in 'map'.
// third template parameter is here to avoid ambiguities
template<class Key, class Elt, class PKey>
static Elt& find(std::map<Key, Elt>& map, const PKey& k)
{
    typename std::map<Key, Elt>::iterator it = map.find( k );
    if( it != map.end() )
        return it->second;
    else
    {
        assert( false );
        return map.begin()->second;
    }
}

// -----------------------------------------------------------------------------

/// Find and retreive an element from the map.
/// @param elt : the key element to found
/// @param res : what is associated with 'elt' in 'map'.
/// @return if we found the element
// third/fourth templates parameters are there to avoid ambiguities
template<class Key, class Elt, class PKey, class PElt>
static bool find(const std::map<Key, Elt>& map, const PKey& elt, PElt const * & res)
{
    typename std::map<Key, Elt>::const_iterator it = map.find( elt );
    if( it != map.end() )
    {
        res = &(it->second);
        return true;
    }
    else
    {
        res = 0;
        return false;
    }
}

// -----------------------------------------------------------------------------

/// Find and retreive an element from the map.
/// @param elt : the key element to found
/// @param res : what is associated with 'elt' in 'map'.
/// @return if we found the element
// third/fourth templates parameters are there to avoid ambiguities
template<class Key, class Elt, class PKey, class PElt>
static bool find(std::map<Key, Elt>& map, const PKey& elt, PElt*& res)
{
    typename std::map<Key, Elt>::iterator it = map.find( elt );
    if( it != map.end() )
    {
        res = &(it->second);
        return true;
    }
    else
    {
        res = 0;
        return false;
    }
}

// -----------------------------------------------------------------------------

/// Find and retreive an element from the map.
/// @param elt : the key element to found
/// @param res : what is associated with 'elt' in 'map'.
/// @return if we found the element
// third template parameter is here to avoid ambiguities
template<class Key, class Elt, class PKey>
static bool exists(const std::map<Key, Elt>& map, const PKey& elt)
{
    return map.find( elt ) != map.end();
}

}// End Namespace Std_utils ====================================================


/**
    @namespace Utils
    @brief Utilities for the c++ programmer everyday life (libs independant)
*/
// =============================================================================
namespace Utils {
// =============================================================================

/// Function macro to generically swap pointers
template<class T> static inline
void swap_pointers(T*& src, T*& dst){
    T* tmp = src;
    src = dst;
    dst = tmp;
}

// -----------------------------------------------------------------------------

template<class T>
void copy(T* dst, const T* src, int nb_elt)
{
    for (int i = 0; i < nb_elt; ++i)
        dst[i] = src[i];
}

// -----------------------------------------------------------------------------

/// A simple integer power function
/// @return x^p
/// @note p must be positive
template<class T>
static inline
T ipow(T x, int p)
{
    assert(p >= 0);
    if (p == 0) return 1;
    if (p == 1) return x;
    return x * ipow(x, p-1);
}

// -----------------------------------------------------------------------------

/// A static version of the integral power
/// @return a^n
template <int n> inline
float ipow(float a) {
    const float b = ipow<n/2>(a);
    return (n & 1) ? (a * b * b) : (b * b);
}

template <> inline float ipow<1>(float a){ return a;   }
template <> inline float ipow<0>(float  ){ return 1.f; }

// -----------------------------------------------------------------------------

/// A static version of the integral power
/// @return a^n
template <int n> inline
int ipow(int a) {
    const int b = ipow<n/2>(a);
    return (n & 1) ? (a * b * b) : (b * b);
}

template <> inline int ipow<1>(int a){ return a; }
template <> inline int ipow<0>(int  ){ return 1; }

// -----------------------------------------------------------------------------


}// End Namespace Utils ========================================================

#endif // STD_UTILS_HPP__
