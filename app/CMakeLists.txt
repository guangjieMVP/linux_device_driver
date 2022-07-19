# CMake 最低版本号要求
cmake_minimum_required (VERSION 2.8)

# 项目信息
project (DogeLog)

#设置C的交叉编译工具
set(CMAKE_C_COMPILER arm-linux-gnueabihf-gcc) 
#设置C++的交叉编译工具
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# 查找当前目录下的所有源文件
# 并将名称保存到 DIR_SRCS 变量
aux_source_directory(. DIR_SRCS)

# 指定生成目标
add_executable(led_test ${DIR_SRCS})
