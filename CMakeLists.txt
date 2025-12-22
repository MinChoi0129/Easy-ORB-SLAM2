cmake_minimum_required(VERSION 3.10)
project(ORB_SLAM2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

# ============================================================================
# Find dependencies
# ============================================================================

# OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV found! Include dirs: ${OpenCV_INCLUDE_DIRS}")
include_directories(${OpenCV_INCLUDE_DIRS})

# Pangolin
find_package(Pangolin REQUIRED)
message(STATUS "Pangolin found! Include dirs: ${Pangolin_INCLUDE_DIRS}")
include_directories(${Pangolin_INCLUDE_DIRS})

# Eigen3
find_package(Eigen3 REQUIRED)
message(STATUS "Eigen3 found! Include dirs: ${EIGEN3_INCLUDE_DIR}")
include_directories(${EIGEN3_INCLUDE_DIR})

# Boost
find_package(Boost REQUIRED COMPONENTS filesystem system serialization)
message(STATUS "Boost found! Include dirs: ${Boost_INCLUDE_DIRS}")
include_directories(${Boost_INCLUDE_DIRS})

# ============================================================================
# Thirdparty libraries
# ============================================================================

# DBoW2
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2)
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2)
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DUtils)

# DBoW2 source files
file(GLOB DBoW2_SRCS
    "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DBoW2/*.cpp"
    "${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/DUtils/*.cpp"
)

# Create DBoW2 library
add_library(DBoW2 STATIC ${DBoW2_SRCS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})
# Add -fPIC for position-independent code (required when linking into shared library)
set_target_properties(DBoW2 PROPERTIES POSITION_INDEPENDENT_CODE ON)

# g2o
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/g2o)
include_directories(${PROJECT_SOURCE_DIR}/Thirdparty/g2o/g2o)

# g2o source files
file(GLOB g2o_SRCS
    "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/g2o/core/*.cpp"
    "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/g2o/types/*.cpp"
    "${PROJECT_SOURCE_DIR}/Thirdparty/g2o/g2o/stuff/*.cpp"
)

# Create g2o library
add_library(g2o STATIC ${g2o_SRCS})
# Eigen3 is header-only, no linking needed
# Add -fPIC for position-independent code (required when linking into shared library)
set_target_properties(g2o PROPERTIES POSITION_INDEPENDENT_CODE ON)

# ============================================================================
# ORB_SLAM2 Library
# ============================================================================

# Include directories
include_directories(${PROJECT_SOURCE_DIR})
include_directories(${PROJECT_SOURCE_DIR}/include)

# Source files
file(GLOB ORB_SLAM2_SRCS
    "${PROJECT_SOURCE_DIR}/src/*.cc"
)

# Create ORB_SLAM2 library
add_library(${PROJECT_NAME} SHARED ${ORB_SLAM2_SRCS})

# Link libraries
target_link_libraries(${PROJECT_NAME}
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
    ${Boost_LIBRARIES}
    DBoW2
    g2o
)

# ============================================================================
# Exampless
# ============================================================================

# Example: binary_vocabulary_creator
add_executable(binary_vocabulary_creator 
    ${PROJECT_SOURCE_DIR}/src/examples/binary_vocabulary_creator.cpp
)
target_link_libraries(binary_vocabulary_creator ${PROJECT_NAME})

# Example: slam_webcam
add_executable(slam_webcam 
    ${PROJECT_SOURCE_DIR}/src/examples/slam_webcam.cpp
)
target_link_libraries(slam_webcam ${PROJECT_NAME})

# Example: slam_video
add_executable(slam_video 
    ${PROJECT_SOURCE_DIR}/src/examples/slam_video.cpp
)
target_link_libraries(slam_video ${PROJECT_NAME})

# Example: relocalization_webcam
add_executable(relocalization_webcam 
    ${PROJECT_SOURCE_DIR}/src/examples/relocalization_webcam.cpp
)
target_link_libraries(relocalization_webcam ${PROJECT_NAME})

# Example: relocalization_video
add_executable(relocalization_video 
    ${PROJECT_SOURCE_DIR}/src/examples/relocalization_video.cpp
)
target_link_libraries(relocalization_video ${PROJECT_NAME})

# ============================================================================
# Compiler-specific settings
# ============================================================================

# Windows-specific settings
if(WIN32)
    add_definitions(-D_CRT_SECURE_NO_DEPRECATE)
    add_definitions(-D_CRT_NONSTDC_NO_DEPRECATE)
endif()

# Linux-specific settings
if(UNIX)
    # Add any Linux-specific compiler flags here if needed
endif()

