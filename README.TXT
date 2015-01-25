To build in Windows, just open msvc/implicit_framework.vcxproj.  If you're not
in Maya 2015 or CUDA 6.5, you may need to update the include and library paths.

Building in Linux with CMake hasn't been tested in a while and will probably
need to be updated.

This requires CUDA with an SM 3 device or better.

Maya quick start:

- Open the script console.
- Load the plugin in the MEL tab:

loadPlugin "/path/to/implicit_framework.mll"

- In the Python tab, add the top directory containing "implicit" to the Python path:

import sys
sys.path.append('/path/to/repository')

- Open a scene and select a mesh.  The mesh must already be skinned.
- In the Python script tab, import and run the main script:

import implicit
implicit.main()

This will create a number of nodes, which will all be placed in a group
named after your skinCluster:

- One ImplicitShape will be created for each skin joint that has painted
weights on the shape.  These are parent constrained to the joints, but
are placed in a separate group to avoid cluttering your bone hierarchy.
These nodes can be seen and viewed in the viewport.  Normally you'll want
to hide them.
- One ImplicitBlend shape will be created, connecting all of the ImplicitShape
nodes together.  This is also shown in the viewport as a merging of all
of the individual ImplicitShapes.  This node isn't parented to anything
and never moves, since it lives in world space.  You'll usually want to
hide this, too.
- An ImplicitDeformer is added to the deformer chain of the original object,
with the ImplicitBlend node as its input.  This performs the actual deformation.











WARNING 1: this is a CUDA project only compatible for Nvidia graphic cards.
The project only guarantee support of GPU card down to compute capability
sm_20 (FERMI architecture). But it might work for architecture below sm_20.

WARNING 2: This is a research application.
It means worse than beta version...
It means clumsy GUI with no undo feature and lots of buttons.
It means it can crash anytime.
You have been warned. Have fun warrior :)

=====================
COMPILING UNDER LINUX
=====================

Be sure to have gcc installed (package build-essential has most dependencies you need).
Be aware that cuda only works with old versions of gcc.
At this time only gcc-4.6 is supported with CUDA 5.0.
so install gcc-4.6 and g++-4.6.
To use other versions you'll need to modify the CMakeList.txt
and replace occurences of gcc-4.xx with the correct compiler version

You also need cmake and make.

Install CudaToolKit from :
https://developer.nvidia.com/cuda-downloads

Don't forget to setup environnement variables in your ~/.bashrc or equivalent:
export LIBRARY_PATH=/usr/lib/nvidia-current:$LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib
export PATH=$PATH:/usr/local/cuda/bin
export CUDA_LIBRARY_PATH=/usr/local/cuda/lib

It is always a good thing to ensure everything is working properly by installing and
compiling the Cuda SDK (same link above). You just need to run make at /CUDA_SDK_PATH/C/common
and then launch a random app in /CUDA_SDK_PATH/C/common/bin

Now that you have checked everything is working install these dependencies :

Library dependencies :
    - glew
    - qt framework 4.7.xx

Building the project is pretty straightforward, just run :
$ cd build
$ cmake -DCUDA_COMPUTE_CAPABILITY=sm_xx ./..
$ make

replace sm_xx by the compute capability of your graphic card (i.e sm_10 sm_11 sm_(..) sm_21 etc.)
you can check the compute capability against your GPU name with wikipedia http://en.wikipedia.org/wiki/CUDA#Supported_GPUs

You can also build the project in debugging mode by doing :

$ cd build
$ cmake -DCUDA_COMPUTE_CAPABILITY=sm_xx -DCMAKE_BUILD_TYPE=Debug ./..
$ make

******************
*Linking problems:
******************
Sometimes your system won't find the libraries /usr/lib/nvidia-currrent/libcuda.so.xxx
which will result in errors at linking for the functions of family cuXXXXX()
The only fix I have found so far is to make symbolic links of these directly within /usr/lib :

ln -s /usr/lib/nvidia-current/libcuda.so    /usr/lib/libcuda.so
ln -s /usr/lib/nvidia-current/libcuda.so.xx libcuda.so.xx
...


=======================
COMPILING UNDER WINDOWS
=======================

Setup under windows is much more complicated, you'd better be ready !

Thirst you need to install visual studio 2010. Add it to your PATH
(e.g "C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\bin").
Prefer the 32 bit version to the 64 bit because the later is really more
of a hassle to get working.

Install the Cuda toolkit from http://developer.nvidia.com/object/cuda_3_2_downloads.html choose
the 32 bit version also you need to have your video card drivers up to date.

Install the Cuda SDK and check if some of the apps are working properly.
Be sure CUDA_SDK_ROOT_DIR environnement variable is set up correctly
(e.g CUDA_SDK_ROOT_DIR = C:\ProgramData\NVIDIA Corporation\NVIDIA GPU Computing SDK 3.2\)

Copy the glew32.lib and GL headers from the SDK into the correct directory of visual

Install QT binnaries for visual studio 2010 (and not mingw nor visual 2008)
don't forget to run the configure.exe located in the root folder of the Qt
binnaries. Also add Qt's /bin directory to your PATH

You also need cmake for windows (http://www.cmake.org/cmake/resources/software.html)
Don't forget to add it to your PATH

-----------------------------
Build under Visual Studio IDE
-----------------------------

Simply  run :

$ cd build
$ call "C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\bin\vcvars32.bat"
$ cmake -DCUDA_COMPUTE_CAPABILITY=sm_x ./..

- Replace the path to your visual studio path.
  vcvars32.bat is needed to setup environment variables before calling visual studio executables through cmake.
- replace sm_x by the compute capability of your graphic card (i.e sm_10 sm_11 sm_(..) sm_21 etc.)

- Open ALL_BUILD.vcproj and stroke F7

----------------------
Build under Qt Creator
----------------------

Add QtCreator to your path in order to jom.exe to be reachable


$ cd build
$ call "C:\Program Files (x86)\Microsoft Visual Studio 10.0\VC\bin\vcvars32.bat"
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_RC_COMPILER="F:/Program Files (x86)/Microsoft_Visual_Studio_10/VC/bin" -DCUDA_COMPUTE_CAPABILITY=sm_x -G"NMake Makefiles JOM" ./..

- Replace the path to your visual studio path.
  vcvars32.bat is needed to setup environment variables before calling visual studio executables through cmake.
- replace sm_x by the compute capability of your graphic card (i.e sm_10 sm_11 sm_(..) sm_21 etc.)
- when specifying path to visual with '-DCMAKE_RC_COMPILER' don't use backslashes but slashes.

- Open Qtcreator
