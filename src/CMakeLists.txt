find_package(yaml-cpp REQUIRED)
find_package(Threads REQUIRED)

add_library(hardware
    SHARED
    hardware.cpp
    joint.cpp
)

target_include_directories(hardware
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(hardware PRIVATE
    ${YAML_CPP_LIBRARIES}
    ${CMAKE_THREAD_LIBS_INIT}
    dxl_x64_cpp
)
