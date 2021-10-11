add_executable(test
    test.cpp
)
target_link_libraries(test PRIVATE ${YAML_CPP_LIBRARIES} hardware)