cmake_minimum_required (VERSION 2.8.7)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

PROJECT(service-client)
set(vsomeip_DIR/home/ygd/vsomeip/build)
#find_package(Threads REQUIRED)
#find_package (vsomeip 2.6.2 REQUIRED)
#find_package( Boost 1.55 COMPONENTS system thread log REQUIRED )
find_package(Threads REQUIRED)

include_directories("/usr/local/include/vsomeip")
link_directories("/usr/local/lib")

add_compile_options( -std=c++11 -O3 -Wall -c -fmessage-length=0 -fPIC -fPIE -pie -fstack-protector-all -Wtrampolines )

include_directories (
    ${Boost_INCLUDE_DIR}
    ${VSOMEIP_INCLUDE_DIRS}
)

    add_executable(service-example ../src/service-example.cpp)
    target_link_libraries(service-example vsomeip3 pthread)
    add_executable (client-example ../src/client-example.cpp)
    target_link_libraries(client-example vsomeip3 pthread)

    add_executable (event_test_client ../src/event_test_client.cpp)
    target_link_libraries(event_test_client vsomeip3 pthread)
    add_executable (event_test_service ../src/event_test_service.cpp)
    target_link_libraries(event_test_service vsomeip3 pthread)



