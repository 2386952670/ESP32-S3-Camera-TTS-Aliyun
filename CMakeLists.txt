cmake_minimum_required(VERSION 3.16)

# 告诉编译器使用自定义分区表
set(PARTITION_TABLE_CUSTOM_FILENAME "partitions.csv")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(camera_web_system)