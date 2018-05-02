
#最小版本的要求
cmake_minimum_required(VERSION 3.0)

#项目名称
project(AnyModelPro)

message("PROJECT_SOURCE_DIR" ${PROJECT_SOURCE_DIR})

#测试主函数的文件夹
add_subdirectory(src test_model)