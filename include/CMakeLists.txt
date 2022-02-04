add_executable(render static.cpp)
add_executable(anim anim.cpp) 


add_definitions( -DMAGICKCORE_QUANTUM_DEPTH=16 )
add_definitions( -DMAGICKCORE_HDRI_ENABLE=1 )
find_package(ImageMagick REQUIRED COMPONENTS Magick++ MagickCore )
include_directories(${ImageMagick_INCLUDE_DIRS})

find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    target_link_libraries(render PRIVATE ${ImageMagick_LIBRARIES} PUBLIC OpenMP::OpenMP_CXX)
    target_link_libraries(anim PRIVATE ${ImageMagick_LIBRARIES} PUBLIC OpenMP::OpenMP_CXX)
else()
    target_link_libraries(render ${ImageMagick_LIBRARIES})
    target_link_libraries(anim ${ImageMagick_LIBRARIES})
endif()
