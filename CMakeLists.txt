project(itkMedialCurveCenterlineExtraction)

cmake_minimum_required(VERSION 2.8)

if(COMMAND cmake_policy)
       cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)
 
INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR})

find_package(ITK REQUIRED)
include(${ITK_USE_FILE})
if (ITKVtkGlue_LOADED)
  find_package(VTK REQUIRED)
  include(${VTK_USE_FILE})
endif()
 
add_executable(itkMedialCurveCenterlineExtraction MACOSX_BUNDLE itkMedialCurveCenterlineExtraction.cpp)


if( "${ITK_VERSION_MAJOR}" LESS 4 )
  target_link_libraries(itkMedialCurveCenterlineExtraction ITKReview ${ITK_LIBRARIES})
else( "${ITK_VERSION_MAJOR}" LESS 4 )
  target_link_libraries(itkMedialCurveCenterlineExtraction ${ITK_LIBRARIES})
endif( "${ITK_VERSION_MAJOR}" LESS 4 )






