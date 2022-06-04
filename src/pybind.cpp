#include <pybind11/pybind11.h>

int add(int i, int j) {
    return i + j;
}

#include "crt/rotations.hpp"
#include "crt/transform.hpp"

#include "crt/cameras.hpp"
#include "crt/entity.hpp"
#include "crt/scene.hpp"
#include "crt/render.hpp"
#include "crt/lighting.hpp"

#include "crt/model_loaders/obj.hpp"
#include "crt/materials/material.hpp"

using Scalar = double;
using Vector3 = bvh::Vector3<Scalar>;

Vector3 create_vec3(Scalar x, Scalar y, Scalar z) {
    return Vector3(x, y, z);
}


PinholeCamera<Scalar> create_pinhole(
    Scalar focal_length,
    pybind11::tuple res,
    pybind11::tuple size,
    Vector3 pos
) {
    struct CameraStruct<Scalar> camera_struct;

    camera_struct.name = "PinholeCamera";

    camera_struct.position = pos;

    camera_struct.rotation[0][0] = Scalar(1);
    camera_struct.rotation[0][1] = Scalar(0);
    camera_struct.rotation[0][2] = Scalar(0);
    camera_struct.rotation[1][0] = Scalar(0);
    camera_struct.rotation[1][1] = Scalar(1);
    camera_struct.rotation[1][2] = Scalar(0);
    camera_struct.rotation[2][0] = Scalar(0);
    camera_struct.rotation[2][1] = Scalar(0);
    camera_struct.rotation[2][2] = Scalar(1);

    camera_struct.focal_length = focal_length;

    camera_struct.resolution[0] = res[0].cast<Scalar>();
    camera_struct.resolution[1] = res[1].cast<Scalar>();

    camera_struct.sensor_size[0] = size[0].cast<Scalar>();
    camera_struct.sensor_size[1] = size[1].cast<Scalar>();

    return PinholeCamera<Scalar>(camera_struct);
}

Entity<Scalar> create_entity(
    Vector3 color,
    std::string path
) {
    Color c;
    c[0] = color[0];
    c[1] = color[1];
    c[2] = color[2];

    
    std::shared_ptr<Material<Scalar>> material(nullptr);
    std::shared_ptr<UVMap<Color>> texture(nullptr);

    texture = std::shared_ptr<UVMap<Color>>(new ConstantUVMap<Color>(c));
    material = std::make_shared<TexturedLambertianMaterial<Scalar>>(texture);

    return Entity<Scalar>(path, material, true);
}

void add_entity(
    Scene<Scalar> &s,
    Entity<Scalar> e,
    Scalar scale,
    Vector3 pos
) {
    
    Scalar rotation[3][3];
    rotation[0][0] = Scalar(1);
    rotation[0][1] = Scalar(0);
    rotation[0][2] = Scalar(0);
    rotation[1][0] = Scalar(0);
    rotation[1][1] = Scalar(1);
    rotation[1][2] = Scalar(0);
    rotation[2][0] = Scalar(0);
    rotation[2][1] = Scalar(0);
    rotation[2][2] = Scalar(1);

    s.add_entity(e, rotation, pos, scale);
}

void foo(Scene<Scalar> &s) {
    (void) s;
}

PYBIND11_MODULE(ceres_rt, m) {
    m.doc() = "ceres ray tracer";
    // m.def("add", &add, "A function that adds two numbers");
    pybind11::class_<Scene<Scalar>>(m, "Scene")
        .def(pybind11::init())
        .def("add_light", &Scene<Scalar>::add_point_light)
        .def("add_light", &Scene<Scalar>::add_square_light)
        .def("add_entity", &add_entity)
        .def("render", [](Scene<Scalar> &s, PinholeCamera<Scalar> &c, std::string path) {
            s.render(c, path);
        })
        .def_readwrite("max_samples", &Scene<Scalar>::max_samples)
        .def_readwrite("min_samples", &Scene<Scalar>::min_samples)
        .def_readwrite("noise_threshold", &Scene<Scalar>::noise_threshold)
        .def_readwrite("num_bounces", &Scene<Scalar>::num_bounces);

    pybind11::class_<Vector3>(m, "Vector3")
    .def(pybind11::init(&create_vec3));

    pybind11::class_<PinholeCamera<Scalar>>(m, "PinholeCamera")
    .def(pybind11::init(&create_pinhole));

    pybind11::class_<PointLight<Scalar>>(m, "PointLight")
    .def(pybind11::init<Vector3, Scalar>());
    
    pybind11::class_<Entity<Scalar>>(m, "Entity")
    .def(pybind11::init(&create_entity));
}