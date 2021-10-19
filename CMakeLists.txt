cmake_minimum_required(VERSION 3.19)
project(VDFilter)

set(CMAKE_CXX_STANDARD 14)

add_executable(VDFilter main.cpp MainVD.cpp MainVD.h Client.cpp Client.h src/TraCIAPI.h src/TraCIAPI.cpp src/socket.h src/socket.cpp src/storage.h src/storage.cpp src/TraCIConstants.h src/TraCIDefs.h)