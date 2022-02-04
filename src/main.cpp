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

#include <Magick++.h> 

#include "render.hpp"
#include "obj.hpp"

template <typename Scalar>
void scene(CameraModel<Scalar> &camera, std::string input_file, std::string out_file, size_t width, size_t height) {
    using Vector3 =  bvh::Vector3<Scalar>;
    using Bvh =  bvh::Bvh<Scalar>;
    using Triangle =  bvh::Triangle<Scalar>;

    Vector3 sun_position = Vector3(100.0, 0, 0.0);

    // Load mesh from file
    auto triangles = obj::load_from_file<Scalar>(input_file);
    if (triangles.size() == 0) {
        std::cerr << "The given scene is empty or cannot be loaded" << std::endl;
        return;
    }

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
    std::cout << "BVH took " << duration.count()/1000000.0 << " s" << std::endl;

    std::cout << "BVH of "
        << bvh.node_count << " node(s) and "
        << reference_count << " reference(s)" << std::endl;

    auto pixels = std::make_unique<Scalar[]>(3 * width * height);

    long tot_rays = 0;

#ifdef _OPENMP
    #pragma omp parallel
    {
        #pragma omp single
        std::cout << "Rendering image on " << omp_get_num_threads() << " threads..." << std::endl;
    }
#else
    std::cout << "Rendering image on single thread..." << std::endl;
#endif

    auto start2 = high_resolution_clock::now();
    auto [rays, hits] = render(camera, sun_position, bvh, triangles.data(), pixels.get(), width, height);
    auto stop2 = high_resolution_clock::now();
    auto duration2 = duration_cast<microseconds>(stop2 - start2);
    tot_rays += rays;
    std::cout << "Rays: " << rays << "\tHits: " << hits << std::endl;
    std::cout << "Tracing took " << duration2.count()/1000000.0 << " s" << std::endl;

    Magick::Image image(Magick::Geometry(width,height), "green");
    image.modifyImage();
    Magick::Pixels view(image);
    Magick::Quantum *img_pix = view.set(0,0,width,height);

    for (size_t j = 0; j < 3 * width * height; j++) {
        *img_pix++ = pixels[j] * 65535;
    }

    view.sync();
    image.write(out_file);

    std::cout << "Total Rays: " << tot_rays << std::endl;    
}


// template <typename Scalar>
// std::unique_ptr<CameraModel<Scalar>> create_camera(const char* camera_model){
//     using Vector3 =  bvh::Vector3<Scalar>;
//     if (!strcmp("PinholeCamera", camera_model) ) {
//         Scalar focal_length = 60;
//         Scalar resolution[2];
//         resolution[0] = 1920;
//         resolution[1] = 1080;
//         Scalar sensor_size[2];
//         sensor_size[0] = 35;
//         sensor_size[1] = 19.7;
//         Vector3 position = Vector3(0.0, -1.0, 0.0);
//         Vector3 euler_angles = Vector3(0.0, 0.0, 0.0);
//         PinholeCamera<Scalar>* camera = new PinholeCamera(focal_length, resolution, sensor_size, position, euler_angles);
//         return camera;
//     }
// }

int main(int argc, char** argv) {

    if (argc != 2) {
        std::cout << "A YAML configuration file must be provided\n";
        return 1;
    }
    else {
        std::cout << "Processing configuration given by: " << argv[1] << "\n";
    }
    
    // Things to be read from the YAML:
    bool use_double = true;
    const char* camera_model = "PinholeCamera";
    size_t width  = 1920;
    size_t height = 1080;
    const char* input_file   = "../data/bunny.obj";
    std::string out_file = "render.png";

    // Parse the camera model:
    if (!strcmp("PinholeCamera", camera_model) ) {
        }
    else {
        std::cout << "Invalid camera model!\n";
    }

    // Build and render the scene as double precision:
    if (use_double) {
        double focal_length = 60;
        double resolution[2];
        resolution[0] = width;
        resolution[1] = height;
        double sensor_size[2];
        sensor_size[0] = 35;
        sensor_size[1] = 19.7;
        using Vector3 = bvh::Vector3<double>;
        Vector3 position = Vector3(0.0, -1.0, 0.0);
        Vector3 euler_angles = Vector3(0.0, 0.0, 0.0);
        PinholeCamera<double> camera(focal_length, resolution, sensor_size, position, euler_angles);
        // auto camera = create_camera<double>(camera_model);
        scene<double>(camera, input_file, out_file, width, height);
    
    // Build and render the scene as single precision:
    } else {
        float focal_length = 60;
        float resolution[2];
        resolution[0] = width;
        resolution[1] = height;
        float sensor_size[2];
        sensor_size[0] = 35;
        sensor_size[1] = 19.7;
        using Vector3 = bvh::Vector3<float>;
        Vector3 position = Vector3(0.0, -1.0, 0.0);
        Vector3 euler_angles = Vector3(0.0, 0.0, 0.0);
        PinholeCamera<float> camera(focal_length, resolution, sensor_size, position, euler_angles);
        // auto camera = create_camera<float>(camera_model);
        scene<float>(camera, input_file, out_file, width, height);
    }
    return 0;
}