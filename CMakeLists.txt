cmake_minimum_required(VERSION 3.10)

# Set the project name
project(AlgotourNative)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Add the executable
add_executable(AlgotourNative src/main.cpp src/GeoTiffLoader.cpp src/GeoTiffLoader.h src/Postprocessor.cpp src/Postprocessor.h src/QueueItem.cpp src/QueueItem.h src/RouteRequest.h src/RouteRequest.cpp)

# Find GDAL package
find_package(GDAL REQUIRED)

# Link GDAL library
target_link_libraries(AlgotourNative ${GDAL_LIBRARIES})

# Include GDAL headers
include_directories(${GDAL_INCLUDE_DIRS})