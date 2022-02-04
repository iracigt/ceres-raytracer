#include <vector>
#include <iostream>
#include <fstream>
#include <cstdint>
#include <chrono>

#include <bvh/bvh.hpp>
#include <bvh/sweep_sah_builder.hpp>
#include <bvh/node_layout_optimizer.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>
#include <bvh/triangle.hpp>

#include <Magick++.h> 

#include "render.hpp"
#include "obj_norms.hpp"

template <typename Scalar>
void scene(std::string input_file, std::string out_file, size_t width, size_t height, int n_frames) {
    using Vector3 =  bvh::Vector3<Scalar>;
    using Bvh =  bvh::Bvh<Scalar>;
    using Triangle =  bvh::Triangle<Scalar>;

    constexpr Scalar pi = Scalar(3.14159265359);

    Camera<Scalar> camera = {
        Vector3(-150.0, -20.0, 0.0),
        Vector3(1, -0.1, 0),
        Vector3(0, -1, 0),
        60
    };

    Scalar rotation_step_degrees = 360.0f/n_frames;
    Vector3 sun_position = Vector3(-500.0, 300.0, 10.0);

    // Load mesh from file
    auto [triangles, tri_norms] = obj::load_from_file<Scalar>(input_file);
    if (triangles.size() == 0) {
        std::cerr << "The given scene is empty or cannot be loaded" << std::endl;
        return;
    }

    std::vector<Magick::Image> frames;

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

    bvh::SweepSahBuilder<Bvh> builder(bvh);
    builder.build(global_bbox, bboxes, centers, reference_count);
    bvh::NodeLayoutOptimizer<Bvh> opt(bvh);
    opt.optimize();
    // std::cout << global_bbox.max[0] << ", " << global_bbox.max[0] << ", " << global_bbox.max[0] << std::endl;

    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);
    std::cout << "BVH took " << duration.count()/1000000.0 << " s" << std::endl;

    std::cout << "BVH of "
        << bvh.node_count << " node(s) and "
        << reference_count << " reference(s)" << std::endl;

    auto pixels = std::make_unique<Scalar[]>(3 * width * height);
    auto t_cam = Transform<Scalar>().rotate(Vector3(0,1,0), rotation_step_degrees / 180.0f * pi);
    auto t_sun = Transform<Scalar>().rotate(Vector3(0,1,0), rotation_step_degrees / 180.0f * pi);

    long tot_rays = 0;

    for (int i = 0; i < n_frames; i++) {
        // Rotate object
        // transform_triangles(t, triangles.data(), triangles.size());

        // Rotate Camera
        camera.eye = t_cam(camera.eye);
        camera.dir = t_cam(camera.dir);
        sun_position = t_sun(sun_position);



#ifdef _OPENMP
        #pragma omp parallel
        {
            #pragma omp single
            std::cout << "Rendering image " << i << " (" << width << "x" << height << ") on " << omp_get_num_threads() << " threads..." << std::endl;
        }
#else
        std::cout << "Rendering image " << i << " (" << width << "x" << height << ") on single thread..." << std::endl;
#endif

        // std::cout << "Rendering image (" << width << "x" << height << ")..." << std::endl;
        auto start2 = high_resolution_clock::now();
        auto [rays, hits] = render(camera, sun_position, bvh, triangles.data(), tri_norms.data(), pixels.get(), width, height);
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
        // image.write("frame_" + std::to_string(i) + ".png");
        image.animationDelay(10);
        frames.push_back(image);
    }

    Magick::writeImages(frames.begin(), frames.end(), out_file);

    std::cout << "Total Rays: " << tot_rays << std::endl;    
}

int main(int argc, char** argv) {

    (void) argc;
    (void) argv;

    bool use_double = false;
    std::string out_file = "render.mp4";

    size_t width  = 1242/2;
    size_t height = 2688/2;
    int n_frames = 60;
    const char* input_file   = "../../waahh_smoothed.obj";

    if (argc > 2) {
        if (!strcmp("-d", argv[1])) {
            use_double = true;   
        }
        out_file = argv[2];
    } else if (argc == 2) {
        out_file = argv[1];
    }

    if (use_double) {
        scene<double>(input_file, out_file, width, height, n_frames);
    } else {
        scene<float>(input_file, out_file, width, height, n_frames);
    }
    
}