cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(samples01 VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 生成された実行ファイルをソースのbinディレクトリに出力する
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(x7_onoff
    src/x7_onoff.cpp
)
target_link_libraries(x7_onoff PRIVATE rt_manipulators_cpp)
