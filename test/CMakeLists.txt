add_executable(test_app
    test.cpp
)
target_link_libraries(test_app PRIVATE ${YAML_CPP_LIBRARIES} hardware)
