cmake_minimum_required(VERSION 3.1...3.20)
project(STLMeshLoader)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

enable_testing()

# CGAL and its components
find_package(CGAL REQUIRED)
find_package(Eigen3 3.3 REQUIRED)  # Ensure Eigen3 is found
# Find Boost with the filesystem component required
find_package(Boost REQUIRED COMPONENTS filesystem)

# Attempt to find the MongoDB C++ driver
find_package(mongocxx REQUIRED)
find_package(bsoncxx REQUIRED)
find_package(Threads REQUIRED)

add_subdirectory(../../public/src ${CMAKE_BINARY_DIR}/public)

# Create the executable
add_executable(STLMeshLoader main_simple.cpp)

# Create a library for MeshEditor
add_library(MeshEditor mesheditor.cpp mesheditor.hpp)
target_include_directories(MeshEditor PUBLIC ${CGAL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
#target_include_directories(MeshEditor PUBLIC ${CGAL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
target_link_libraries(MeshEditor CGAL::CGAL Boost::filesystem)

#add libraries
add_library(MongoDBClient MongoDBClient.cpp MongoDBClient.h)

add_library(stlviewer stlviewer.cpp stlviewer.h)
target_link_libraries(stlviewer fssimplewindow ysclass)


# Link CGAL and Boost.Filesystem
target_link_libraries(MongoDBClient mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads)
#target_link_libraries(STLMeshLoader CGAL::CGAL Boost::filesystem mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads MongoDBClient stlviewer fssimplewindow ysclass MeshEditor)
target_link_libraries(STLMeshLoader CGAL::CGAL Boost::filesystem mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads MongoDBClient stlviewer fssimplewindow ysclass MeshEditor)

add_executable(segmentation_tests main_test.cpp)
target_link_libraries(segmentation_tests CGAL::CGAL Boost::filesystem mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads MongoDBClient stlviewer fssimplewindow ysclass MeshEditor)
add_test(NAME SimpleMeshTest COMMAND segmentation_tests simple)
add_test(NAME ComplexMeshTest COMMAND segmentation_tests complex)
add_test(NAME DegenerateMeshTest COMMAND segmentation_tests degenerate)








# cmake_minimum_required(VERSION 3.1...3.20)
# project(STLMeshLoader)

# # Specify the C++ standard
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED True)

# # CGAL and its components
# find_package(CGAL REQUIRED)
# # Find Boost with the filesystem component required
# find_package(Boost REQUIRED COMPONENTS filesystem)

# # Attempt to find the MongoDB C++ driver
# find_package(mongocxx REQUIRED)
# find_package(bsoncxx REQUIRED)
# find_package(Threads REQUIRED)

# add_subdirectory(../../public/src ${CMAKE_BINARY_DIR}/public)

# # Create the executable
# add_executable(STLMeshLoader main_simple.cpp)

# # Create a library for MeshEditor
# add_library(MeshEditor mesheditor.cpp mesheditor.hpp)
# target_include_directories(MeshEditor PUBLIC ${CGAL_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
# target_link_libraries(MeshEditor CGAL::CGAL Boost::filesystem)

# #add libraries
# add_library(MongoDBClient MongoDBClient.cpp MongoDBClient.h)

# add_library(stlviewer stlviewer.cpp stlviewer.h)
# target_link_libraries(stlviewer fssimplewindow ysclass)


# # Link CGAL and Boost.Filesystem
# target_link_libraries(MongoDBClient mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads)
# #target_link_libraries(STLMeshLoader CGAL::CGAL Boost::filesystem mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads MongoDBClient stlviewer fssimplewindow ysclass MeshEditor)
# target_link_libraries(STLMeshLoader CGAL::CGAL Boost::filesystem mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads MongoDBClient stlviewer fssimplewindow ysclass MeshEditor)