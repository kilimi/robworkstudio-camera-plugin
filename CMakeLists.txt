CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
# The name of the project.
PROJECT(lulu_winz)

# Used to resolve absolute path names
SET(ROOT ${CMAKE_CURRENT_SOURCE_DIR})

# Now set the RW/RWS root (edit this if necessary)
SET(RW_ROOT "${ROOT}/../../../RobWork")
SET(RWS_ROOT "${ROOT}/../../../RobWorkStudio")

# We use the settings that robwork studio uses
SET(CMAKE_BUILD_TYPE Release)

SET(RobWorkStudio_DIR "${RWS_ROOT}/cmake") 
FIND_PACKAGE(RobWorkStudio)

INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} ${ROBWORKSTUDIO_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} ${ROBWORKSTUDIO_LIBRARY_DIRS} )

find_package(PCL 1.3 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package( OpenCV REQUIRED )



#########################   From here we add the plugins 

# if we want to use ui files add them here
SET(UIS_FILES LuluWinz.ui)
RWS_QT4_WRAP_UI(UIS_OUT_H ${UIS_FILES})
SET(SrcFiles lulu_winz.cpp ${UIS_OUT_H})

QT4_WRAP_CPP(MocSrcFiles lulu_winz.hpp OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED)
QT4_ADD_RESOURCES(RccSrcFiles resources.qrc)

message(${PCL_LIBRARIES})
  # The shared library to build:
ADD_LIBRARY(lulu_winz MODULE ${SrcFiles} ${MocSrcFiles}  ${RccSrcFiles})
TARGET_LINK_LIBRARIES(lulu_winz ${ROBWORKSTUDIO_LIBRARIES} ${ROBWORK_LIBRARIES})
target_link_libraries(lulu_winz  ${PCL_LIBRARIES})
target_link_libraries( lulu_winz ${OpenCV_LIBS} )