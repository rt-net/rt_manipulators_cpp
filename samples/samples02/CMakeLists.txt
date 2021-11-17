cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(samples02 VERSION 1.0)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 生成された実行ファイルをソースのbinディレクトリに出力する
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# サンプルのビルド
set(list_samples
    x7_forwared_kinematics
)
foreach(sample IN LISTS list_samples)
    message("${sample}")
    add_executable(${sample}
        src/${sample}.cpp
    )
    target_link_libraries(${sample} PRIVATE rt_manipulators_cpp)
endforeach()
