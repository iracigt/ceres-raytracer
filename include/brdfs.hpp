#ifndef __BRDFS_H
#define __BRDFS_H

#include <bvh/bvh.hpp>

template <typename Scalar>
void lambertian(const bvh::Ray<Scalar> &light_ray, const bvh::Vector3<Scalar> &normal, bvh::Vector3<Scalar> &intensity, const bvh::Vector3<Scalar> &color){
    Scalar light_intensity = 1;
    Scalar L_dot_N = light_ray.direction[0]*normal[0] + light_ray.direction[1]*normal[1] +light_ray.direction[2]*normal[2];
    intensity = L_dot_N*color*light_intensity;
};

template <typename Scalar>
bvh::Ray<Scalar> cosine_importance(bvh::Vector3<Scalar> origin, bvh::Vector3<Scalar> normal, Scalar r1, Scalar r2) {
    // This implementation is based on Malley's method:

    // Generate a random sample:
    Scalar r = std::sqrt(r1);
    Scalar theta = r2*2*M_PI;
    Scalar x = r*std::cos(theta);
    Scalar y = r*std::sin(theta);
    Scalar z = std::sqrt(std::max(0.0, 1.0 - r1*r1 - r2*r2));

    // Transform back into world coordinates:
    bvh::Vector3<Scalar> Nx = bvh::Vector3<Scalar>(normal[2],0.0,-normal[0])*Scalar(1.0/std::sqrt(normal[0]*normal[0] + normal[2]*normal[2]));
    // std::cout<< Nx[0] << "\n";
    bvh::Vector3<Scalar> Ny = cross(normal, Nx);
    // std::cout<< Ny[0] << "\n";
    bvh::Vector3<Scalar> dir(x*Ny[0] + y*normal[0] + z*Nx[0], 
                             x*Ny[1] + y*normal[1] + z*Nx[1], 
                             x*Ny[2] + y*normal[2] + z*Nx[2]);
    // bvh::Vector3<Scalar> dir(r1/2 + normal[0], r2/2 + normal[1], z/2 + normal[2]);
    return bvh::Ray<Scalar>(origin, bvh::normalize(dir));
}

#endif