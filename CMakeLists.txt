cmake_minimum_required(VERSION 3.26)
project("spnipu" LANGUAGES CXX)

include(CTest)
include(ExternalProject)

add_compile_options(-fno-limit-debug-info)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED On)

cmake_policy(SET CMP0167 NEW)
set(Boost_USE_STATIC_LIBS OFF) 
set(Boost_USE_MULTITHREADED ON)  
set(Boost_USE_STATIC_RUNTIME OFF) 
find_package(Boost REQUIRED)

add_subdirectory(libspnipu)
add_subdirectory(apps)