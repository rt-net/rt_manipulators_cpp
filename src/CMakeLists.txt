find_package(yaml-cpp REQUIRED)

add_library(hardware
    SHARED
    hardware.cpp
)

target_include_directories(hardware
    PUBLIC ${PROJECT_SOURCE_DIR}/include
)
target_link_libraries(hardware PRIVATE ${YAML_CPP_LIBRARIES} dxl_x64_cpp)