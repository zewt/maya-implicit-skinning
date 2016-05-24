###############################################################################
# Warning: This file is not process by qmake, it is solely used as a convenient
# way to work with qt creator IDE.
# Build is handle exclusively with CMake please go to the CMakeLists.txt file
# to change build configuration
###############################################################################

SOURCES += \
    src/animation/*.* \
    src/animation/pinocchio/*.* \
    src/blending_lib/*.* \
    src/blending_lib/cuda_interface/*.* \
    src/control/*.* \
    src/containers/*.* \
    src/global_datas/*.* \
    src/hrbf/*.* \
    src/maths/*.* \
    src/parsers/*.* \
    src/qt_gui/common/*.* \
    src/qt_gui/common/customize/*.* \
    src/qt_gui/common/popup/*.* \
    src/qt_gui/common/toolbars/*.* \
    src/qt_gui/common/widgets/*.* \
    src/qt_gui/common/gl_widgets/*.* \
    src/qt_gui/common/gizmo2/*.* \
    src/qt_gui/common/gizmo_deprecated/*.* \
    src/qt_gui/CSG/*.* \
    src/qt_gui/SKIN/*.* \
    src/qt_gui/SKIN_V2/*.* \
    src/implicit_graphs/*.* \
    src/implicit_graphs/CSG/*.* \
    src/implicit_graphs/skinning/*.* \
    src/primitives/*.* \
    src/primitives/hrbf/*.* \
    src/primitives/precomputed_prim/*.* \
    src/rendering/*.* \
    src/rendering/environment_map/*.* \
    src/scene_tree/*.* \
    src/utils/*.* \
    src/utils/gl_utils/*.* \
    src/utils/cuda_utils/*.* \
    src/meshes/vcg_lib/*.* \
    src/meshes/voxelizer/*.* \
    src/meshes/*.* \
    src/maya/*.* \
    src/*.*

#-------------------------------------------------------------------------------
# SHADERS
SOURCES +=  src/shaders/*

#-------------------------------------------------------------------------------

OTHER_FILES += CMakeLists.txt

FORMS += \
    include/qt_gui/common/*.ui \
    include/qt_gui/common/customize/*.ui \
    include/qt_gui/common/popup/*.ui \
    include/qt_gui/common/widgets/*.ui \
    include/qt_gui/common/toolbars/*.ui \
    include/qt_gui/CSG/*.ui \
    include/qt_gui/SKIN/*.ui \
    include/qt_gui/CSG/tree.ui \
    include/qt_gui/SKIN_V2/*.ui \
    include/qt_gui/common/gl_widgets/spline_popup.ui \
    include/qt_gui/common/gl_widgets/operator_name_dialog.ui



################################################################################

# Some hacks for qt Creator to correctly highlight code
# and activate autocompletion for the librairies used in this project

QT += opengl

INCLUDEPATH += \
    libs/include \
    libs/include/vcglib \
    include \
    src \
    src/animation \
    src/blending_functions \
    src/blending_lib \
    src/blending_lib/cuda_interface \
    src/control \
    src/containers \
    src/gizmo \
    src/global_datas \
    src/maths \
    src/maths/intersections \
    src/parsers \
    src/pinocchio \
    src/primitives \
    src/primitives/hrbf \
    src/primitives/precomputed_prim \
    src/qt_gui \
    src/implicit_graphs \
    src/implicit_graphs/CSG \
    src/implicit_graphs/skinnning \
    src/rendering \
    src/utils \
    src/utils/portable_srcs \
    src/utils/gl_utils \
    src/utils/cuda_utils \
    src/meshes/voxelizer \
    src/meshes/vcg_lib \
    src/meshes \
    src/scene_tree

TARGET = implicit_skinning

# QT CUDA TRICKS
#(autocompletion with cuda)
unix:INCLUDEPATH += /usr/local/cuda/include
unix:INCLUDEPATH += /usr/include/qt4
unix:INCLUDEPATH += /usr/include/qt4/QtGUI
unix:INCLUDEPATH += /usr/include/c++/4.4
unix:INCLUDEPATH += /usr/include/c++/
win32:INCLUDEPATH += "C:\CUDA\include"
win32:INCLUDEPATH += "F:\Program Files\NVIDIA GPU Computing Toolkit\CUDA\v4.2\include"
win32:INCLUDEPATH += "F:\ProgramData\NVIDIA Corporation\NVIDIA GPU Computing SDK 4.2\C\common\inc"
win32:INCLUDEPATH += "F:\ProgramData\NVIDIA Corporation\NVIDIA GPU Computing SDK 4.2\C\common\inc"

win32:INCLUDEPATH += "C:\Program Files\Autodesk\Maya2015\include"
INCLUDEPATH += libs/include

#(avoid unrecognized keywords)
DEFINES += "__device__="
DEFINES += "__host__="
DEFINES += "__global__="
DEFINES += "__shared__="
DEFINES += "__constant__="
