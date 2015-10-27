#ifndef IDENTIFIER_HPP
#define IDENTIFIER_HPP

#include "cuda_compiler_interop.hpp"

/**
 * @class Identifier
 * @brief An integer type safe index
 *
 * To access an array one need a integer indentifier. Some time to add semantic
 * and inform the user which kind of index is to be used you will do
 * @code
 *  typedef int My_id;
 * @endcode
 *
 * Which is bad design because typedef are NOT type safe
 * (its merely a name alias), compiler will throw no warnings nor errors with
 * implicit conversion. Its not as bad as no typedefs
 *
 * The solution consist to encapsulate the integer into the class provided here.
 * With this class an identifier interacts with identifiers of the same only
 * type and integers.
 *
 * An accessor defined like this:
 * @code
 *     value get_value const (Specific_id i);
 * @endcode
 * will force the user to use the correct type 'Specific_id'.
 * However we keep the simplicity of integer indexing by allowing every
 * arithmetic operations with identifier of the same type and
 * (unsigned)/intergers.
 *
 * Use case :
 * @code
 * // Declare a type safe identifier :
 * DEFINE_IDENTIFIER(Example0);
 * DEFINE_IDENTIFIER(Example1);
 * // You can use Example0 and Example1 as a regular integer with operations
 * // like +,-,/, -= etc.
 *
 * Val get0(Example0 a);
 * Val get1(Example1 a);
 *
 * void main()
 * {
 *     // Declare two different identifiers
 *     Example0 idx_t0( 10 );
 *     Example1 idx_t1( 11 );
 *     int array[N];
 *
 *     // will fail:
 *     array[ idx_t0 + idx_t0 ]; // Identifiers are not integers
 *     array[ (idx_t0 + idx_t1).id() ]; // Identifiers have different types
 *     get0( idx_t1 ); // Argument does not match identifier type
 *     get0( 10 ); // Integers must be explicitly converted
 *
 *     // will work:
 *     idx_t0 += idx_t0; // arithmetic with same identifiers type allowed
 *     idx_t0 += 2; // arithmetic with integers allowed
 *     array[ (idx_t0 + idx_t0).id() ];
 *     get0( idx_t0 );
 *     get1( idx_t1 );
 *     get0( Example0(10) ); // Explicit integer conversion with constructor
 * }
 * @endcode
 *
 * You can give a quite simple interface to look up an array without the need
 * of verbose iterators:
 *
 * @code
 * #include <vector>
 *
 * struct My_struct {
 *     DEFINE_IDENTIFIER(Id);
 *
 *     Val& get_elt(Id i) { return _array[i.id()]; }
 *     int nb_elt const () { return _array.size(); }
 * private:
 *     std::vector<Vals> _array;
 * };
 *
 * {
 *     int offset = 2;
 *     My_struct s;
 *     for(int i = 0; i < s.nb_elt() - offset; ++i)
 *         s.get_elt( My_struct::Id(0) + i + offset);
 * }
 * @endcode
 *
 * Another Idea of interface could be
 * @code
 * #include <vector>
 *
 * struct My_struct {
 *     DEFINE_IDENTIFIER(Id);
 *
 *     Val& get_elt(Id i) { return _array[i.id()]; }
 *     Id begin() const () { return Id(0);             }
 *     Id end()   const () { return Id(_array.size()); }
 * private:
 *     std::vector<Vals> _array;
 * };
 *
 * {
 *     int offset = 2;
 *     My_struct s;
 *     for(My_struct::Id = s.begin(); i != (s.end() - offset); ++i)
 *         s.get_elt( i + offset);
 * }
 * @endcode
 *
 */
template<class Id_name>
class Identifier {
protected:
    int _id;
public:
    explicit IF_CUDA_DEVICE_HOST Identifier(unsigned id) : _id( (int)id ) {}
    explicit IF_CUDA_DEVICE_HOST Identifier(int id) : _id( id ) {}
    IF_CUDA_DEVICE_HOST Identifier() : _id( -1 ) {}

    // -------------------------------------------------------------------------
    /// @name Comparaison operators
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline bool operator==(const Id_name& i) const { return _id == i._id; }
    IF_CUDA_DEVICE_HOST inline bool operator!=(const Id_name& i) const { return _id != i._id; }
    IF_CUDA_DEVICE_HOST inline bool operator<=(const Id_name& i) const { return _id <= i._id; }
    IF_CUDA_DEVICE_HOST inline bool operator>=(const Id_name& i) const { return _id >= i._id; }
    IF_CUDA_DEVICE_HOST inline bool operator< (const Id_name& i) const { return _id <  i._id; }
    IF_CUDA_DEVICE_HOST inline bool operator> (const Id_name& i) const { return _id >  i._id; }

    // -------------------------------------------------------------------------
    /// @name Arythmetic between identifiers
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline Id_name operator +(const Id_name& i) const { return Id_name(_id  + i._id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator -(const Id_name& i) const { return Id_name(_id  - i._id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator *(const Id_name& i) const { return Id_name(_id  * i._id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator /(const Id_name& i) const { return Id_name(_id  / i._id); }

    IF_CUDA_DEVICE_HOST inline Id_name operator= (const Id_name& i) { return Id_name(_id  = i._id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator+=(const Id_name& i) { return Id_name(_id += i._id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator-=(const Id_name& i) { return Id_name(_id -= i._id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator*=(const Id_name& i) { return Id_name(_id *= i._id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator/=(const Id_name& i) { return Id_name(_id /= i._id); }

    // -------------------------------------------------------------------------
    /// @name Arythmetic between identifiers and integers
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline Id_name operator +(int i) const { return Id_name(_id  + i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator -(int i) const { return Id_name(_id  - i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator *(int i) const { return Id_name(_id  * i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator /(int i) const { return Id_name(_id  / i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator+=(int i) { return Id_name(_id += i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator-=(int i) { return Id_name(_id -= i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator*=(int i) { return Id_name(_id *= i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator/=(int i) { return Id_name(_id /= i); }

    // -------------------------------------------------------------------------
    /// @name Arythmetic between identifiers and unsigned integers
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline Id_name operator +(unsigned i) const { return Id_name(_id  + (int)i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator -(unsigned i) const { return Id_name(_id  - (int)i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator *(unsigned i) const { return Id_name(_id  * (int)i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator /(unsigned i) const { return Id_name(_id  / (int)i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator+=(unsigned i) { return Id_name(_id += (int)i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator-=(unsigned i) { return Id_name(_id -= (int)i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator*=(unsigned i) { return Id_name(_id *= (int)i); }
    IF_CUDA_DEVICE_HOST inline Id_name operator/=(unsigned i) { return Id_name(_id /= (int)i); }

    // -------------------------------------------------------------------------
    /// @name (post/pre) (in/de)crementation
    // -------------------------------------------------------------------------

    IF_CUDA_DEVICE_HOST inline Id_name operator ++(   ) { return Id_name(++_id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator ++(int) { return Id_name(_id++); }
    IF_CUDA_DEVICE_HOST inline Id_name operator --(   ) { return Id_name(--_id); }
    IF_CUDA_DEVICE_HOST inline Id_name operator --(int) { return Id_name(_id--); }

    IF_CUDA_DEVICE_HOST inline bool is_valid() const { return _id > -1; }
    IF_CUDA_DEVICE_HOST inline int  id()       const { return _id;      }
};

/// Define an integer typedef with strong type check.
/// For more explanations on how to use this macro goto Identifier doc
/// @see Identifier
#define DEFINE_IDENTIFIER( Class_name ) \
    struct Class_name : public Identifier<Class_name> { \
        explicit IF_CUDA_DEVICE_HOST Class_name(int id) : Identifier<Class_name>(id) { } \
        IF_CUDA_DEVICE_HOST Class_name() : Identifier<Class_name>(-1) { } \
}

#endif // IDENTIFIER_HPP
