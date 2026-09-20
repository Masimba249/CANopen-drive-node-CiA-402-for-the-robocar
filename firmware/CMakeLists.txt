# ESP-IDF project: robocar CANopen drive node (CiA 402).
# Tested against ESP-IDF v5.1 - v5.3 on ESP32.
#
#   idf.py set-target esp32
#   idf.py build flash monitor
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(robocar_canopen_drive)
