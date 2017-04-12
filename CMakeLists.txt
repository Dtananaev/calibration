cmake_minimum_required (VERSION 2.6 FATAL_ERROR)
#set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
#MESSAGE("FALGS: ${CMAKE_CXX_FLAGS}")
project      (visualizer)
find_package (Qt4 REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)
find_package(JPEG)
find_package(PNG)





include_directories (${PCL_INCLUDE_DIRS}
                     ./include
                     ./external/png/include
                     .//external/others/include)
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES  ./src/main.cpp ./src/pclviewer.cpp ./src/loader.cpp)
set  (project_HEADERS ./include/pclviewer.h ./include/loader.h)
set  (project_FORMS   ./src/pclviewer.ui)
set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT4_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT4_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (calibration ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (calibration ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${JPEG_LIBRARIES} ${PNG_LIBRARIES})



