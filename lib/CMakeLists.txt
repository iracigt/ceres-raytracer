add_library(
    lib
    INIReader.h
    happly.hpp
    lodepng/lodepng.cpp
    lodepng/lodepng.h
)

add_subdirectory(bvh)

set(CMAKE_CXX_FLAGS "-fPIC") #???

set_target_properties(lib PROPERTIES LINKER_LANGUAGE CXX)

target_include_directories(bvh PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")