# ----------------------------------------------------------------------------
#  Root CMake file for Ouster Lidar Project
# ----------------------------------------------------------------------------
#
# Example CMake build line for windows & VS2015
# windows: cmake -G "Visual Studio 14 2015 Win64" -DUSE_AVX_INSTRUCTIONS=ON ..
# linux: cmake -DUSE_AVX_INSTRUCTIONS=ON ..
#
#
# To compile:
# windows: cmake --build . --config Release
# linux: cmake --build . --config Release -- -j10
#

cmake_minimum_required(VERSION 2.8.12)

PROJECT(Lidar_Project)
set(exe_name "ouster_client")

set(PLATFORM "$ENV{PLATFORM}" CACHE INTERNAL "Copied from environment variable")

message("Platform: " ${PLATFORM})

if(${PLATFORM} STREQUAL "Laptop")
    set(COMMON_ROOT "D:/Common")

elseif(${PLATFORM} STREQUAL "MainGear")
    set(COMMON_ROOT "~/Common")

elseif(${PLATFORM} STREQUAL "HPC")
    set(COMMON_ROOT "$HOME/Common")
endif()

include(${COMMON_ROOT}/cmake_utils/use_cpp_11.cmake)
include(${COMMON_ROOT}/cmake_utils/use_sse_avx.cmake)

#include_directories("include")

# grab all of the *.c and *.cpp files in the src directory
file(GLOB SOURCES "src/*.cpp" 
        )

set(project_includes "include" "${COMMON_ROOT}/include")
            
message("")
message("--------------------------------------------------------")
message("Source Files: ")
foreach(S ${SOURCES})
    message("  " ${S})
endforeach()
message("--------------------------------------------------------")
message("")

# find OpenCV
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
	set(project_includes ${project_includes} ${OpenCV_INCLUDE_DIRS})
    
    message(" ")
    message("--------------------------------------------------------")
    message("Project Includes: ")
    foreach(S ${project_includes})
        message("  " ${S})
    endforeach()
    message("--------------------------------------------------------")
    message(" ")
    
    include_directories(${project_includes})
	add_executable(${exe_name} ${SOURCES})
	target_link_libraries(${exe_name} ${OpenCV_LIBS})

else()
	message("OpenCV not found, so we won't build the project.")
endif()

if(WIN32)
    message("MSVC Version: " ${MSVC_VERSION})
    # uncomment for MSVC and cmake greater than 3.6.0
    if(MSVC AND (${CMAKE_VERSION} VERSION_GREATER "3.6.0"))
        set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} /MP)
        set(CMAKE_C_FLAGS ${CMAKE_C_FLAGS} /MP)
        set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${exe_name})
    endif()
endif()

message("Build Complete!")
message(" ")
