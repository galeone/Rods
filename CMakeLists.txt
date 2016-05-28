cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)

project( Rods CXX)
find_package( OpenCV REQUIRED )

add_executable( Rods main.cpp rod.cpp hole.cpp ) 
target_link_libraries( Rods ${OpenCV_LIBS} )

target_compile_features(Rods PRIVATE cxx_auto_type cxx_return_type_deduction)
