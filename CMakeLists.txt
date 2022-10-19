cmake_minimum_required (VERSION 3.5)

project (demo)

set (EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) ## bin is the EXECUTABLE_OUTPUT_PATH

set (SRC_LIST ${PROJECT_SOURCE_DIR}/src/main.cpp)

include_directories (/usr/local/include/)
include_directories (/usr/local/include/opencascade)

include (${PROJECT_SOURCE_DIR}/FindEIGEN.cmake)
INCLUDE_DIRECTORIES (${EIGEN_INCLUDE_DIRS})

INCLUDE_DIRECTORIES (${PROJECT_SOURCE_DIR}/include)
INCLUDE_DIRECTORIES ($ENV{HOME}/pkg/gmsh-4.8.4-source/MY_GMSH/include)
#INCLUDE_DIRECTORIES ($ENV{HOME}/gmsh_test/gmsh-better-entity-remove/MY_GMSH/include)

find_library(GMSH_LIB gmsh HINTS $ENV{HOME}/pkg/gmsh-4.8.4-source/MY_GMSH/lib)

# OpenCASCADE

INCLUDE (${PROJECT_SOURCE_DIR}/FindOpenCASCADE.cmake)
INCLUDE_DIRECTORIES (${OCC_INCLUDE_DIR})
#

#########below use a LIB: OPENMP
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb -DDEBUG")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")

FIND_PACKAGE( OpenMP REQUIRED) #find omp
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS}${OpenMP_EXE_LINKER_FLAGS}")
endif()
#########OPENMP

add_executable (main ${SRC_LIST})

target_link_libraries (main ${GMSH_LIB})

target_link_libraries (main ${OCC_LIBRARIES})