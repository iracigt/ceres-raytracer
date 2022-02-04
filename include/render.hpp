#ifndef __RENDER_H
#define __RENDER_H

#include <cstdint>
#include <algorithm>

#include <bvh/bvh.hpp>
#include <bvh/binned_sah_builder.hpp>
#include <bvh/single_ray_traverser.hpp>
#include <bvh/primitive_intersectors.hpp>
#include <bvh/triangle.hpp>

#include "render.hpp"
#include "transform.hpp"

template <typename Scalar>
struct Camera {
    bvh::Vector3<Scalar> eye;
    bvh::Vector3<Scalar> dir;
    bvh::Vector3<Scalar> up;
    Scalar  fov;
};

template <size_t Axis, typename Scalar>
static void rotate_triangles(Scalar degrees, bvh::Triangle<Scalar>* triangles, size_t triangle_count) {
    static constexpr Scalar pi = Scalar(3.14159265359);
    auto cos = std::cos(degrees * pi / Scalar(180));
    auto sin = std::sin(degrees * pi / Scalar(180));
    auto rotate = [&] (const bvh::Vector3<Scalar>& p) {
        if (Axis == 0)
            return bvh::Vector3<Scalar>(p[0], p[1] * cos - p[2] * sin, p[1] * sin + p[2] * cos);
        else if (Axis == 1)
            return bvh::Vector3<Scalar>(p[0] * cos + p[2] * sin, p[1], -p[0] * sin + p[2] * cos);
        else
            return bvh::Vector3<Scalar>(p[0] * cos - p[1] * sin, p[0] * sin + p[1] * cos, p[2]);
    };
    #pragma omp parallel for
    for (size_t i = 0; i < triangle_count; ++i) {
        auto p0 = rotate(triangles[i].p0);
        auto p1 = rotate(triangles[i].p1());
        auto p2 = rotate(triangles[i].p2());
        triangles[i] = bvh::Triangle<Scalar>(p0, p1, p2);
    }
}

template <typename Scalar>
float lambertian(bvh::Vector3<Scalar> sun_line, bvh::Vector3<Scalar> normal){
    return std::abs(sun_line[0]*normal[0] + sun_line[1]*normal[1] + sun_line[2]*normal[2]);
};

template <typename Scalar>
float blinn_phong_spec(bvh::Vector3<Scalar> sun_line, bvh::Vector3<Scalar> normal, bvh::Vector3<Scalar> view) {
    return std::pow(bvh::dot(normal, bvh::normalize(sun_line + view)), 24);
};

template <typename Scalar>
std::array<float, 3> smooth_shading(bvh::Vector3<Scalar> sun_line, std::array<bvh::Vector3<Scalar>, 3> norms, bvh::Vector3<Scalar> view, Scalar u, Scalar v) {
    
    std::array c{0.f,0.f,0.f};
    // Calculate the shading:
    float amb = 0.2;
    float diffuse;
    float specular;

    diffuse = 0.5f * lambertian(sun_line, norms[0]);
    specular = 0.8f * blinn_phong_spec(sun_line, norms[0], view * Scalar(-1.0));
    c[0] += u * std::clamp((amb + diffuse) * 0.5f + specular, 0.f, 1.f);
    c[1] += u * std::clamp((amb + diffuse) * 0.0f + specular, 0.f, 1.f);
    c[2] += u * std::clamp((amb + diffuse) * 0.8f + specular, 0.f, 1.f);

    diffuse = 0.5f * lambertian(sun_line, norms[1]);
    specular = 0.8f * blinn_phong_spec(sun_line, norms[1], view * Scalar(-1.0));
    c[0] += v * std::clamp((amb + diffuse) * 0.5f + specular, 0.f, 1.f);
    c[1] += v * std::clamp((amb + diffuse) * 0.0f + specular, 0.f, 1.f);
    c[2] += v * std::clamp((amb + diffuse) * 0.8f + specular, 0.f, 1.f);

    diffuse = 0.5f * lambertian(sun_line, norms[2]);
    specular = 0.8f * blinn_phong_spec(sun_line, norms[2], view * Scalar(-1.0));
    c[0] += (1-u-v) * std::clamp((amb + diffuse) * 0.5f + specular, 0.f, 1.f);
    c[1] += (1-u-v) * std::clamp((amb + diffuse) * 0.0f + specular, 0.f, 1.f);
    c[2] += (1-u-v) * std::clamp((amb + diffuse) * 0.8f + specular, 0.f, 1.f);

    return c;
}

template <typename Scalar>
std::pair<int, int> render(const Camera<Scalar>& camera, const bvh::Vector3<Scalar>& sun_position, const bvh::Bvh<Scalar>& bvh,
            const bvh::Triangle<Scalar>* triangles, std::array<bvh::Vector3<Scalar>, 3> *tri_norms, Scalar* pixels,
            size_t width, size_t height)
{
    auto dir = bvh::normalize(camera.dir);
    auto image_u = bvh::normalize(bvh::cross(dir, camera.up));
    auto image_v = bvh::normalize(bvh::cross(image_u, dir));
    auto image_w = std::tan(camera.fov * Scalar(3.14159265 * (1.0 / 180.0) * 0.5));
    auto ratio = Scalar(height) / Scalar(width);
    image_u = image_u * image_w;
    image_v = image_v * image_w * ratio;

    bvh::ClosestPrimitiveIntersector<bvh::Bvh<Scalar>, bvh::Triangle<Scalar>, false> intersector(bvh, triangles);
    bvh::SingleRayTraverser<bvh::Bvh<Scalar>> traverser(bvh);

    size_t traversal_steps = 0, intersections = 0;

    #pragma omp parallel for collapse(2) reduction(+: traversal_steps, intersections)
    for(size_t i = 0; i < width; ++i) {
        for(size_t j = 0; j < height; ++j) {
            size_t index = 3 * (width * j + i);

            auto u = 2 * (i + Scalar(0.5)) / Scalar(width)  - Scalar(1);
            auto v = 2 * (j + Scalar(0.5)) / Scalar(height) - Scalar(1);
            auto view = bvh::normalize(image_u * u + image_v * v + dir);

            bvh::Ray<Scalar> ray(camera.eye, view);
            auto hit = traverser.traverse(ray, intersector);
            traversal_steps++;
            if (!hit) {
                pixels[index] = pixels[index + 1] = pixels[index + 2] = 0;
            } else {
                intersections++;
                auto ind = hit->primitive_index;
                auto tri = triangles[hit->primitive_index];
                auto normal = bvh::normalize(tri.n);
                // pixels[index    ] = std::fabs(normal[0]);
                // pixels[index + 1] = std::fabs(normal[1]);
                // pixels[index + 2] = std::fabs(normal[2]);

                auto u = hit->intersection.u;
                auto v = hit->intersection.v;
                bvh::Vector3<Scalar> intersect_point = (u*tri.p0 + v*tri.p1() + (1-u-v)*tri.p2());

                //TODO: Figure out how to deal with the self-intersection stuff in a more proper way...
                Scalar scale = -0.00001;
                intersect_point = intersect_point + scale*normal;

                bvh::Vector3<Scalar> sun_line = bvh::normalize(sun_position - intersect_point);
                bvh::Ray<Scalar> ray(intersect_point, sun_line);
                auto hit = traverser.traverse(ray, intersector);
                traversal_steps++;
                if (!hit) {

                    // Calculate the shading:
                    auto c = smooth_shading(sun_line, tri_norms[ind], view, u, v);
                    
                    pixels[index    ] = c[0];
                    pixels[index + 1] = c[1];
                    pixels[index + 2] = c[2];
                } else{
                    intersections++;
                    pixels[index] = pixels[index + 1] = pixels[index + 2] = 0;
                }
            }
        }
    }

    return std::pair(traversal_steps, intersections);
}

#endif