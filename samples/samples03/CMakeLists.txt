cmake_minimum_required(VERSION 3.10)

# set the project name and version
project(samples03 VERSION 1.0)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 生成された実行ファイルをソースのbinディレクトリに出力する
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# サンプルのビルド
set(list_samples
    x7_3dof_inverse_kinematics
    s17_3dof_inverse_kinematics
    x7_gravity_compensation
    s17_gravity_compensation
)
foreach(sample IN LISTS list_samples)
    message("${sample}")
    add_executable(${sample}
        src/${sample}.cpp
        src/rt_manipulators_ik.cpp
        src/rt_manipulators_dynamics.cpp
    )
    target_include_directories(${sample} PUBLIC
        include
    )
    target_link_libraries(${sample} PRIVATE rt_manipulators_cpp Eigen3::Eigen)
endforeach()
