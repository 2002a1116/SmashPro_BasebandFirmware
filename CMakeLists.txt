
# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
add_compile_options("-Wno-format")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(bluetooth)
# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量