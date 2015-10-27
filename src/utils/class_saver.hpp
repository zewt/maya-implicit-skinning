#ifndef CLASS_SAVER_HPP__
#define CLASS_SAVER_HPP__

#include <iostream>
#include <fstream>
#include <string>

// -----------------------------------------------------------------------------

/// Load a class instance in the user allocated area pointed by 'inst' from the
/// file designated by path
template <class T>
void load_class(T* inst, const std::string& path)
{
    std::ifstream istream( path.c_str(), std::ios::in | std::ios::binary );

    if( !istream.is_open() )
        std::cerr << "File "<< path <<" could not be opened." << std::endl;
    else
        istream.read(reinterpret_cast<char*>( inst ), sizeof(*inst) );

    istream.close();
}


// -----------------------------------------------------------------------------

/// Save the class instance 'inst' to a file indicated by 'path'. File is
/// created or override
template <class T>
void save_class(const T* inst, const std::string& path)
{
    std::ofstream ostream( path.c_str(), std::ios::trunc | std::ios::out | std::ios::binary );

    if( !ostream.is_open() )
        std::cerr << "File "<< path <<" could not be opened." << std::endl;
    else
        ostream.write( reinterpret_cast<const char*>( inst ), sizeof(*inst) );

    ostream.close();
}

// -----------------------------------------------------------------------------

/// @param ptr pointer to the newly allocated array
/// @param size nb elements of the array
/// @param file_path
/// @return wether the file exist or not
template<class T>
bool read_array(T* ptr, int size, std::string file_path)
{
    std::ifstream istream(file_path.c_str(), std::ios::in | std::ios::binary);

    if( !istream.is_open() )
        return false;

    istream.read(reinterpret_cast<char*>( ptr ), sizeof(T)* size);

    istream.close();
    return true;
}

// -----------------------------------------------------------------------------

/// @return wether the file has been written or not
template<class T>
bool write_array(const T* ptr, int size, std::string file_path)
{
    std::ofstream ostream(file_path.c_str(), std::ios::trunc | std::ios::out | std::ios::binary );

    if( !ostream.is_open() )
        return false;

    ostream.write(reinterpret_cast<const char*>( ptr ), sizeof(T)* size);

    ostream.close();
    return true;
}

// -----------------------------------------------------------------------------

#endif // CLASS_SAVER_HPP__
