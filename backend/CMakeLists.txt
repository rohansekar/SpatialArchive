cmake_minimum_required(VERSION 3.10)
project(BackendServer)
enable_testing()

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Attempt to find the MongoDB C++ driver
find_package(mongocxx REQUIRED)
find_package(bsoncxx REQUIRED)
find_package(Threads REQUIRED)

# Add the executable
add_executable(BackendServer backend_main.cpp)
add_library(Backend backend.cpp backend.h)

# For Rohan's Program
add_executable(Program main.cpp)
add_library(MongoDBClient MongoDBClient.cpp MongoDBClient.h)
add_executable(Program_test program_test.cpp)
add_executable(Backend_test backend_test.cpp)

# Link the libraries using the imported targets from the find_package command
target_link_libraries(MongoDBClient mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads)
target_link_libraries(Program PRIVATE mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads MongoDBClient)
target_link_libraries(Backend mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads)
target_link_libraries(BackendServer mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads Backend)
#target_link_libraries(BackendServer PRIVATE mongo::mongocxx_shared mongo::bsoncxx_shared Threads::Threads)
target_link_libraries(Program_test mongo::mongocxx_shared mongo::bsoncxx_shared MongoDBClient)
target_link_libraries(Backend_test mongo::mongocxx_shared mongo::bsoncxx_shared Backend)

# For Rohan's Program
add_test(NAME Program_test COMMAND Program_test)

add_test(NAME Backend_test COMMAND Backend_test)
