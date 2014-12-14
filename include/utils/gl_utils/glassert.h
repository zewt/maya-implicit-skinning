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
#ifndef GL_ASSERT_H
#define GL_ASSERT_H

#ifndef NDEBUG
//#include <GL/glu.h>
#include <iostream>
#include <cassert>

/** @brief debugging tool for opengl calls

    This file provide macros in order to check easily openGl API calls.
    use it on every call to ensure better correctness of your programm :
    @code
    #include "glassert.h"

    glAssert( glmyAPICall() )
    @endcode

    @warning never use a glAssert() on a begin() directive.
    glGetError is not allowed inside a glBegin gEnd block !

    On can track down opengl errors with GL_CHECK_ERRORS(); which is usefull if
    the project don't use glAssert() everywhere. In this case a glAssert() can
    be triggered by previous error and won't give valuable insight on where the
    problem is located and from what opengl's primitive call.
*/

#define __TO_STR(x) __EVAL_STR(x)
#define __EVAL_STR(x) #x

#define glAssert(code) do{code; int l = __LINE__;\
   GLuint err = glGetError(); \
                if (err != GL_NO_ERROR)\
                { \
                  std::cerr << "OpenGL error : " << __FILE__ << "\n";\
                  std::cerr << "line : " << l << "\n";\
                  std::cerr << "Source code : " << __TO_STR(code) << "\n";\
                  std::cerr << "Message : " << (const char*)gluErrorString(err) << "("<<err<<")" << "\n";\
                  assert(false); \
              }\
}while(false)

// -----------------------------------------------------------------------------

#define GL_CHECK_ERRORS() \
do{ GLuint err = glGetError(); \
                if (err != GL_NO_ERROR)\
                { \
                  std::cerr << "OpenGL error : " << __FILE__ << "\n";\
                  std::cerr << "line : " << __LINE__ << "\n";\
                  std::cerr << "Source code : " << __TO_STR(code) << "\n";\
                  std::cerr << "Message : " << (const char*)gluErrorString(err) << "("<<err<<")" << "\n";\
                  assert(false); \
              }\
}while(false)

#else

#define GL_CHECK_ERRORS()

#define glAssert(code) \
    code
#endif


#endif
