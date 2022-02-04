#include <vector>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <chrono>

#include <bvh/bvh.hpp>
#include <bvh/binned_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>
#include <bvh/triangle.hpp>

#include "render.hpp"
#include "obj.hpp"

using Scalar = float;
using Triangle = bvh::Triangle<Scalar>;
using Vector3 = bvh::Vector3<Scalar>;
using Bvh = bvh::Bvh<Scalar>;

int main(int argc, char** argv) {

    (void) argc;
    (void) argv;

    const char* output_file  = "render.ppm";
    // const char* input_file   = "Bennu_v20_200k.obj";
    // Camera camera = {
    //     Vector3(0.0,  -1.0, 0.0),
    //     Vector3(0, 1, 0),
    //     Vector3(0, 0, 1),
    //     60
    // };
    // Vector3 sun_position = Vector3(100.0, 0, 0.0);
    // size_t rotation_axis = 0;
    // Scalar rotation_degrees = 0;

    const char* input_file = "../../data/dragon.obj";
    Camera<Scalar> camera = {
        Vector3(0.0, -15.0, 2.0),
        Vector3(0, 1, 0),
        Vector3(0, 0, 1),
        60
    };
    Vector3 sun_position = Vector3(-50.0, -20.0, 0.0);
    size_t rotation_axis = 0;
    Scalar rotation_degrees = 90;

    // const char* input_file = "dragon.obj";
    // Camera camera = {
    //     Vector3(-2.0, 0.0, 0.0),
    //     Vector3(1, 0, 0),
    //     Vector3(0, 0, 1),
    //     60
    // };
    // Vector3 sun_position = Vector3(-50.0, 100.0, 0.0);
    // size_t rotation_axis = 0;
    // Scalar rotation_degrees = 90;

    // const char* input_file = "bike.obj";
    // Camera camera = {
    //     Vector3(-2.0, 0.0, 0.5),
    //     Vector3(1, 0, 0),
    //     Vector3(0, 0, 1),
    //     60
    // };
    // Vector3 sun_position = Vector3(-50.0, 100.0, 0.0);
    // size_t rotation_axis = 0;
    // Scalar rotation_degrees = 90;
    

    size_t width  = 1920;
    size_t height = 1080;

    // Load mesh from file
    auto triangles = obj::load_from_file<Scalar>(input_file);
    if (triangles.size() == 0) {
        std::cerr << "The given scene is empty or cannot be loaded" << std::endl;
        return 1;
    }

    // Rotate triangles if requested
    if (rotation_axis == 0)
        rotate_triangles<0>(rotation_degrees, triangles.data(), triangles.size());
    else if (rotation_axis == 1)
        rotate_triangles<1>(rotation_degrees, triangles.data(), triangles.size());
    else if (rotation_axis == 2)
        rotate_triangles<2>(rotation_degrees, triangles.data(), triangles.size());

    Bvh bvh;

    size_t reference_count = triangles.size();
    std::unique_ptr<Triangle[]> shuffled_triangles;

    // Build an acceleration data structure for this object set
    std::cout << "Building BVH ( using BinnedSahBuilder )..." << std::endl;
    using namespace std::chrono;
    auto start = high_resolution_clock::now();

    auto bboxes_and_centers = bvh::compute_bounding_boxes_and_centers(triangles.data(), triangles.size());
    auto bboxes = bboxes_and_centers.first.get(); 
    auto centers = bboxes_and_centers.second.get(); 
    
    auto global_bbox = bvh::compute_bounding_boxes_union(bboxes, triangles.size());

    bvh::BinnedSahBuilder<Bvh, 16> builder(bvh);
    builder.build(global_bbox, bboxes, centers, reference_count);

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << duration.count()/1000000.0 << std::endl;

    std::cout << "BVH of "
        << bvh.node_count << " node(s) and "
        << reference_count << " reference(s)" << std::endl;

    auto pixels = std::make_unique<Scalar[]>(3 * width * height);

#ifdef _OPENMP
        #pragma omp parallel
        {
            #pragma omp single
            std::cout << "Rendering image (" << width << "x" << height << ") on " << omp_get_num_threads() << " threads..." << std::endl;
        }
#else
        std::cout << "Rendering image (" << width << "x" << height << ") on single thread..." << std::endl;
#endif
    // std::cout << "Rendering image (" << width << "x" << height << ")..." << std::endl;
    auto start2 = high_resolution_clock::now();
    render(camera, sun_position, bvh, triangles.data(), nullptr, pixels.get(), width, height);
    auto stop2 = high_resolution_clock::now();
    auto duration2 = duration_cast<microseconds>(stop2 - start2);
    std::cout << duration2.count()/1000000.0 << std::endl;

    std::ofstream out(output_file, std::ofstream::binary);
    out << "P6 " << width << " " << height << " " << 255 << "\n";
    for(size_t j = height; j > 0; --j) {
        for(size_t i = 0; i < width; ++i) {
            size_t index = 3* (width * (j - 1) + i);
            uint8_t pixel[3] = {
                static_cast<uint8_t>(std::max(std::min(pixels[index    ] * 255, Scalar(255)), Scalar(0))),
                static_cast<uint8_t>(std::max(std::min(pixels[index + 1] * 255, Scalar(255)), Scalar(0))),
                static_cast<uint8_t>(std::max(std::min(pixels[index + 2] * 255, Scalar(255)), Scalar(0)))
            };
            out.write(reinterpret_cast<char*>(pixel), sizeof(uint8_t) * 3);
        }
    }
}