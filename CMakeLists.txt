cmake_minimum_required(VERSION 3.15)
project(fast_lio CXX)

# ===========================================
# Build Configuration
# ===========================================
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

# ===========================================
# Platform-specific Configuration
# ===========================================
if(WIN32)
    message(STATUS "=== Building on Windows ===")
    add_definitions(-DWIN32 -D_WIN32 -DWINDOWS_COMPAT -DNOMINMAX)
    # PCL alignment fix
    add_definitions(-DPCL_SILENCE_MALLOC_WARNING=1)
    add_definitions(-DEIGEN_MAX_ALIGN_BYTES=32)
    # Boost placeholder warning fix
    add_definitions(-DBOOST_BIND_GLOBAL_PLACEHOLDERS)
    # PCL binary IO compatibility (disable deprecated functions)
    add_definitions(-DPCL_NO_PRECOMPILE)
    
    if(MSVC)
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /O2 /EHsc /MP /openmp")
    else()
        # MinGW
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -fopenmp -fexceptions")
    endif()
    
    # Library paths for Windows
    set(LIBRARY_BASE_PATH "E:/Library_save" CACHE PATH "Base path for libraries")
    set(PCL_ROOT "${LIBRARY_BASE_PATH}/USE_For_PCL/PCL/PCL 1.15.1" CACHE PATH "Root of PCL installation")
    set(PCL_INCLUDE_DIRS "${PCL_ROOT}/include/pcl-1.15")
    set(PCL_LIBRARY_DIRS "${PCL_ROOT}/lib")
    set(EIGEN3_INCLUDE_DIR "${LIBRARY_BASE_PATH}/Eigen/eigen-5.0.0")
    set(BOOST_INCLUDEDIR "${PCL_ROOT}/3rdParty/Boost/include/boost-1_87")
    set(BOOST_LIBRARYDIR "${PCL_ROOT}/3rdParty/Boost/lib")
    set(FLANN_INCLUDE_DIRS "${PCL_ROOT}/3rdParty/FLANN/include")
    set(FLANN_LIBRARY_DIRS "${PCL_ROOT}/3rdParty/FLANN/lib")
    set(QHULL_LIBRARY_DIRS "${PCL_ROOT}/3rdParty/Qhull/lib")
else()
    # Linux/macOS
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -pthread -fexceptions")
    add_compile_options(-fdeclspec)
    find_package(OpenMP REQUIRED)
    find_package(Eigen3 REQUIRED)
    find_package(PCL REQUIRED COMPONENTS common io filters kdtree)
endif()

# ===========================================
# Source Files
# ===========================================
set(FASTLIO_MAPPING_SOURCES
    src/laserMapping.cpp
    src/preprocess.cpp
    src/conversions.cpp
    include/ikd-Tree/ikd_Tree.cpp
)

# ===========================================
# Create Library
# ===========================================
add_library(fastlio_mapping SHARED ${FASTLIO_MAPPING_SOURCES})

set_target_properties(fastlio_mapping PROPERTIES
    POSITION_INDEPENDENT_CODE ON
    WINDOWS_EXPORT_ALL_SYMBOLS ON
)

# ===========================================
# Include Directories
# ===========================================
target_include_directories(fastlio_mapping PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/compat
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

if(WIN32)
    # VTK includes (needed for lz4 used by FLANN)
    set(VTK_INCLUDE_DIRS "${PCL_ROOT}/3rdParty/VTK/include/vtk-9.4")
    set(LZ4_INCLUDE_DIRS "${VTK_INCLUDE_DIRS}/vtklz4/lib")
    
    target_include_directories(fastlio_mapping PUBLIC
        ${BOOST_INCLUDEDIR}
        ${FLANN_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
        ${LZ4_INCLUDE_DIRS}
    )
endif()

# ===========================================
# Link Libraries
# ===========================================
if(WIN32)
    target_link_directories(fastlio_mapping PUBLIC
        ${PCL_LIBRARY_DIRS}
        ${BOOST_LIBRARYDIR}
        ${FLANN_LIBRARY_DIRS}
        ${QHULL_LIBRARY_DIRS}
    )
    
    target_link_libraries(fastlio_mapping PUBLIC
        pcl_common pcl_io pcl_io_ply pcl_filters pcl_kdtree
        pcl_search pcl_features pcl_octree pcl_sample_consensus
        libboost_filesystem-vc143-mt-x64-1_87
        libboost_system-vc143-mt-x64-1_87
        libboost_thread-vc143-mt-x64-1_87
        libboost_iostreams-vc143-mt-x64-1_87
        flann_cpp_s
        ws2_32 wsock32
    )
else()
    if(OpenMP_CXX_FOUND)
        target_link_libraries(fastlio_mapping PUBLIC OpenMP::OpenMP_CXX)
    endif()
    target_link_libraries(fastlio_mapping PUBLIC
        ${PCL_COMMON_LIBRARIES}
        ${PCL_IO_LIBRARIES}
        ${PCL_FILTERS_LIBRARIES}
        ${PCL_KDTREE_LIBRARIES}
    )
endif()

# ===========================================
# Main Executable (fastlio_process)
# ===========================================
add_executable(fastlio_process
    src/main.cpp
    src/data_reader.cpp
)

target_include_directories(fastlio_process PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/include/compat
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

if(WIN32)
    target_include_directories(fastlio_process PRIVATE
        ${BOOST_INCLUDEDIR}
        ${FLANN_INCLUDE_DIRS}
        ${VTK_INCLUDE_DIRS}
        ${LZ4_INCLUDE_DIRS}
    )
endif()

target_link_libraries(fastlio_process PRIVATE fastlio_mapping)

# ===========================================
# Installation
# ===========================================
install(TARGETS fastlio_mapping fastlio_process
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
)
