cmake_minimum_required(VERSION 3.5)

project(vSomeIPHelloWorld)
find_package(Threads REQUIRED)

include_directories("/usr/local/include/vsomeip")
link_directories("/usr/local/lib")
cmake_minimum_required(VERSION 3.5)

PROJECT(bin)

# Find required protobuf package
find_package(Protobuf REQUIRED)
if(PROTOBUF_FOUND)
    message(STATUS "protobuf library found")
else()
    message(FATAL_ERROR "protobuf library is needed but cant be found")
endif()

include_directories(${PROTOBUF_INCLUDE_DIRS})

include_directories("/usr/include/")

add_compile_options( -std=c++11 -O3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines )

add_executable (test1 test1.cpp message.pb.cc)
target_link_libraries(test1  vsomeip3 pthread ${PROTOBUF_LIBRARIES} )
add_executable (test2 test2.cpp message.pb.cc)
target_link_libraries(test2  vsomeip3 pthread ${PROTOBUF_LIBRARIES} )

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin) 
