#
# This is a CMake makefile.  You can find the cmake utility and
# information about it at http://www.cmake.org
#

cmake_minimum_required(VERSION 2.8.4)

PROJECT(EvanTracking)

include(../dlib/cmake)

find_package(OpenCV)
if (OpenCV_FOUND)
   include_directories(${OpenCV_INCLUDE_DIRS})
   #if code use opencv,must add in here
   ADD_EXECUTABLE(EvanTracking EvanTracking.cpp)
   TARGET_LINK_LIBRARIES(EvanTracking dlib ${OpenCV_LIBS} )
endif()

