cmake_minimum_required (VERSION 2.6 FATAL_ERROR)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
MESSAGE("FALGS: ${CMAKE_CXX_FLAGS}")
project      (calibration)
find_package (VTK REQUIRED)
find_package (PCL REQUIRED)
find_package(JPEG)
find_package(PNG)
find_package (Qt5Widgets REQUIRED)
find_package (Qt5Core REQUIRED)



include_directories (${PCL_INCLUDE_DIRS}
                     ./include
                     ./external/png/include
                     .//external/others/include)
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

set  (project_SOURCES  ./src/main.cpp ./src/pclviewer.cpp ./src/loader.cpp ./src/tools.cpp ./src/calibration.cpp ./external/others/include/NMath.cpp)
set  (project_HEADERS ./include/pclviewer.h ./include/loader.h ./include/tools.h ./include/calibration.h )
set  (project_FORMS   ./src/pclviewer.ui)
#set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})
set(QT_LIBRARIES Qt5::Widgets Qt5::Core)
#INCLUDE         (${QT_USE_FILE})
#ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (calibration ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (calibration ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${JPEG_LIBRARIES} ${PNG_LIBRARIES})


ADD_EXECUTABLE  (tool ./src/tools.cpp ./include/tools.h ./src/toolsApp.cpp ./external/others/include/NMath.cpp)

TARGET_LINK_LIBRARIES (tool)

ADD_EXECUTABLE  (im2g ./src/jpg2pgm.cpp )
TARGET_LINK_LIBRARIES (im2g ${JPEG_LIBRARIES}  ${QT_LIBRARIES} )

ADD_EXECUTABLE  (calib ./src/calibTest.cpp  ./src/calibration.cpp ./src/loader.cpp ./src/tools.cpp ./external/others/include/NMath.cpp )
TARGET_LINK_LIBRARIES (calib  ${JPEG_LIBRARIES} ${QT_LIBRARIES} )

