#
# This is a CMake makefile.  You can find the cmake utility and
# information about it at http://www.cmake.org
#

cmake_minimum_required(VERSION 2.8.4)

PROJECT(examples)

include(../dlib/cmake)

# Since there are a lot of examples I'm going to use a macro to simply this
# CMakeLists.txt file.  However, usually you will create only one executable in
# your cmake projects and use the syntax shown above.
MACRO(add_example name)
   ADD_EXECUTABLE(${name} ${name}.cpp)
   TARGET_LINK_LIBRARIES(${name} dlib )
ENDMACRO()

#here we apply our macros 
add_example(video_tracking_ex)

find_package(OpenCV)
if (OpenCV_FOUND)
   include_directories(${OpenCV_INCLUDE_DIRS})
endif()

if (DLIB_LINK_WITH_SQLITE3)
   add_example(sqlite_ex)
endif()


