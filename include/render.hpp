#ifndef __RENDER_H
#define __RENDER_H

#include <cstdint>
#include <math.h>

#include <bvh/bvh.hpp>
#include <bvh/binned_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>
#include <bvh/triangle.hpp>

#include "lights.hpp"
#include "brdfs.hpp"
#include "cameras.hpp"

template <typename Scalar>
bvh::Vector3<Scalar> illumination(bvh::SingleRayTraverser<bvh::Bvh<Scalar>> &traverser, 
                                 bvh::ClosestPrimitiveIntersector<bvh::Bvh<Scalar>, bvh::Triangle<Scalar>, false> &intersector, 
                                 Scalar &u, Scalar &v, bvh::Triangle<Scalar> &tri, bvh::Ray<Scalar> light_ray, bvh::Vector3<Scalar> view) {
    bvh::Vector3<Scalar> intensity(0.0,0.0,0.0);;
    // Loop through all provided lights:
    auto hit = traverser.traverse(light_ray, intersector);
    if (!hit) {
        //TODO: Add smooth shading as an option per triangle:
        bvh::Vector3<Scalar> interpolated_normal = u*tri.vn1 + v*tri.vn2 + (1-u-v)*tri.vn0;
        bvh::Vector3<Scalar> interpolated_color = 
        u*bvh::Vector3<Scalar>(tri.vc[1]) +
        v*bvh::Vector3<Scalar>(tri.vc[2]) + 
        (1-u-v)*bvh::Vector3<Scalar>(tri.vc[0]);
        
        float amb = 0.0;
        float diffuse = lambertian(light_ray.direction, interpolated_normal);
        float spec = blinn_phong_spec(light_ray.direction, interpolated_normal, view);
        // bvh::Vector3<Scalar> color(0.5, 0.0, 0.8);

        intensity = interpolated_color * Scalar(amb + diffuse) + bvh::Vector3<Scalar>(1) * Scalar(spec);

    }

    for (int i = 0; i < 3; i++) {
        intensity[i] = std::clamp(intensity[i], Scalar(0.0), Scalar(1.0));
    }

    return intensity;
}


template <typename Scalar>
bvh::Vector3<Scalar>  direct_lighting(std::vector<PointLight<Scalar>> point_lights, bvh::Vector3<Scalar> intersect_point, Scalar &u, Scalar &v,
                     bvh::Triangle<Scalar> &tri, bvh::SingleRayTraverser<bvh::Bvh<Scalar>> &traverser, bvh::Vector3<Scalar> view,
                     bvh::ClosestPrimitiveIntersector<bvh::Bvh<Scalar>, bvh::Triangle<Scalar>, false> &intersector) {
    bvh::Vector3<Scalar> intensity;
    int count = 1;
    for (PointLight<Scalar> light : point_lights){

        bvh::Ray<Scalar> light_ray = light.sample_ray(intersect_point);
        bvh::Vector3<Scalar> intensity_new = illumination<Scalar>(traverser, intersector, u, v, tri, light_ray, view);
        if (count == 1) {
            intensity = intensity_new;
        }
        else {
            intensity[0] += (intensity_new[0] - intensity[0])/count;
            intensity[1] += (intensity_new[1] - intensity[1])/count;
            intensity[2] += (intensity_new[2] - intensity[2])/count;
        }
        count++;
    };
    //TODO Add in area lights:
    // for AreaLight<Scalar> light : area_lights){
    // };
    return intensity;
};


template <typename Scalar>
void render(CameraModel<Scalar> &camera, std::vector<PointLight<Scalar>> point_lights, const bvh::Bvh<Scalar>& bvh,
            const bvh::Triangle<Scalar>* triangles, Scalar* pixels)
{
    bvh::ClosestPrimitiveIntersector<bvh::Bvh<Scalar>, bvh::Triangle<Scalar>, false> intersector(bvh, triangles);
    bvh::SingleRayTraverser<bvh::Bvh<Scalar>> traverser(bvh);

    // Initialize random number generator:
    std::random_device rd;
    std::default_random_engine eng(rd());
    std::uniform_real_distribution<Scalar> distr(0.0, 1.0);

    // TODO: MOVE THIS TO BE PARAMETER OF RENDER:
    int num_samples = 5;
    bool no_bounce = false;

    size_t width  = (size_t) floor(camera.get_resolutionX());
    size_t height = (size_t) floor(camera.get_resolutionY());

    auto bg_color = bvh::Vector3<Scalar>(135, 206, 250) * Scalar(1./255.);

    #pragma omp parallel for
    for(size_t i = 0; i < width; ++i) {
        for(size_t j = 0; j < height; ++j) {
            size_t index = 3 * (width * j + i);
            // Loop through all samples for a given pixel:
            // TODO: Make this adaptive sampling at some point...
            bvh::Vector3<Scalar> pixel_intensity(0.0,0.0,0.0);
            for (int sample = 1; sample < num_samples+1; ++sample) {
                // TODO: Make a better random sampling algorithm:
                bvh::Ray<Scalar> ray;
                if (num_samples == 1) {
                    ray = camera.pixel_to_ray(i, j);
                }
                else {
                    ray = camera.pixel_to_ray(i + distr(eng), j + distr(eng));
                }
                auto view = ray.direction * Scalar(-1.0);
                bvh::Vector3<Scalar> new_intensity;
                auto hit = traverser.traverse(ray, intersector);
                if (!hit) {
                    new_intensity = bg_color;
                } 
                else {
                    auto tri = triangles[hit->primitive_index];

                     // TODO: Move this normalization into the triangle creation step.
                    auto normal = bvh::normalize(tri.n);

                    if (no_bounce) {
                        new_intensity[0] = normal[0];
                        new_intensity[1] = normal[1];
                        new_intensity[2] = normal[2];
                    }
                    else {
                        //TODO: Figure out how to deal with the self-intersection stuff in a more proper way...
                        auto u = hit->intersection.u;
                        auto v = hit->intersection.v;
                        bvh::Vector3<Scalar> intersect_point = (u*tri.p1() + v*tri.p2() + (1-u-v)*tri.p0);
                        Scalar scale = -0.0000001;
                        intersect_point = intersect_point + scale*normal;

                        // Calculate the direct lighting:
                        auto direct_intensity = direct_lighting<Scalar>(point_lights, intersect_point, u, v, tri, traverser, view, intersector);

                        // Calculate indirect lighting:
                        new_intensity = direct_intensity;
                    }
                }

                // Update the new pixel intensity:
                pixel_intensity += (new_intensity - pixel_intensity) * Scalar(1.0 / sample);
            }
            // Store the pixel intensity:
            pixels[index    ] = std::fabs(pixel_intensity[0]);
            pixels[index + 1] = std::fabs(pixel_intensity[1]);
            pixels[index + 2] = std::fabs(pixel_intensity[2]);
        }
    }
}

#endif