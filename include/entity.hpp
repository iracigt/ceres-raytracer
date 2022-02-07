#ifndef __ENTITY_H
#define __ENTITY_H

#include <memory>
#include <vector>
#include <random>

#include <bvh/bvh.hpp>
#include <bvh/triangle.hpp>
#include <bvh/vector.hpp>

#include <Magick++.h>

#include "obj.hpp"
#include "happly.hpp"
#include "rotations.hpp"
#include "transform.hpp"

using Color = bvh::Vector3<float>;

template <typename Value>
class UVMap {
    public:
    virtual Value operator()(float u, float v) = 0;
};

template <typename Value>
class ConstantUVMap : public  UVMap<Value> {
    private:
    Value v;
   
    public:
    ConstantUVMap(Value val) : v(val) { }
    Value operator()(float u, float v) { return v; };
};

class ImageUVMap : public UVMap<Color> {
    private:
    
    Magick::Image img;
    
    public:
    ImageUVMap(std::string path) { img.read(path); }
    
    Color operator()(float u, float v) {
        size_t x = (size_t)(u * img.size().width() + 0.5);
        size_t y = (size_t)(v * img.size().height() + 0.5);

        Magick::Color color = img.pixelColor(x, y);

        return Color(
            color.quantumRed() / 65535.f,
            color.quantumGreen() / 65535.f,
            color.quantumBlue() / 65535.f
        );
    };
};

template <typename Scalar>
class Material {
    public:
    virtual Color get(bvh::Ray<Scalar> light_ray, bvh::Vector3<Scalar> normal, float u, float v) = 0;
};

template <typename Scalar>
class ColoredLambertianMaterial : public Material<Scalar> {
    private:
    Color c;
    
    public:
    ColoredLambertianMaterial(Color color) : c(color) { }

    Color get(bvh::Ray<Scalar> light_ray, bvh::Vector3<Scalar> normal, float u, float v) {
        auto L_dot_N = bvh::dot(light_ray.direction, normal);
        return c * (float)(L_dot_N);
    }
};

template <typename Scalar>
class TexturedLambertianMaterial : public Material<Scalar> {
    private:
    std::shared_ptr<UVMap<Color>> tex_map;
   
    public:
    TexturedLambertianMaterial(std::shared_ptr<UVMap<Color>> texture) : tex_map(texture) { }

    Color get(bvh::Ray<Scalar> light_ray, bvh::Vector3<Scalar> normal, float u, float v) {
        auto L_dot_N = bvh::dot(light_ray.direction, normal);
        return (*tex_map)(u, v) * (float)(L_dot_N);
    }
};


template <typename Scalar>
class Entity {
    using Triangle = bvh::Triangle<Scalar>;
    using Vector3 = bvh::Vector3<Scalar>;

    private:        

        std::vector<Triangle> triangles;
        std::vector<std::shared_ptr<Material<Scalar>>> materials;
        std::shared_ptr<UVMap<size_t>> material_map;

    public:

        Entity() = delete;

        Entity(std::string path, std::string texture = "", Color color=Color(0.5, 0.5, 0.5)) {

            material_map = std::shared_ptr<UVMap<size_t>>(new ConstantUVMap<size_t>(0));
            
            if (!texture.empty()) {
                materials.emplace_back(new TexturedLambertianMaterial<Scalar>(std::shared_ptr<UVMap<Color>>(new ImageUVMap(texture))));
            } else { 
                materials.emplace_back(std::shared_ptr<Material<Scalar>>(new ColoredLambertianMaterial<Scalar>(color)));
            }
            
            bool is_ply = path.size() > 4 && 0 == path.compare(path.size() - 4, 4, ".ply");
            
            if (!is_ply) {
                auto tris = obj::load_from_file<Scalar>(path);
                
                for (auto &tri : tris) {
                    Triangle t(tri.p0, tri.p1(), tri.p2());
                    t.set_parent(this);
                    t.add_vetex_normals(tri.vn0, tri.vn1, tri.vn2);
                    triangles.push_back(t);
                }
            } else {
                // Construct the data object by reading from file
                happly::PLYData plyIn(path);

                // Get mesh-style data from the object
                auto vPos = plyIn.getVertexPositions();
                auto fInd = plyIn.getFaceIndices<size_t>();
                bool color = plyIn.getElement("vertex").hasProperty("red");
                auto vColor = color ? plyIn.getVertexColors() : std::vector<std::array<unsigned char, 3>>();
                bool tex = plyIn.getElement("vertex").hasProperty("s");
                auto texX = tex ? plyIn.getElement("vertex").getProperty<float>("s") : std::vector<float>();
                auto texY = tex ? plyIn.getElement("vertex").getProperty<float>("t") : std::vector<float>();

                std::vector<bvh::Vector3<Scalar> > normals(vPos.size(), bvh::Vector3<Scalar>(0,0,0));    

                for (auto face : fInd) {
                    if (face.size() != 3) {
                        throw std::invalid_argument("PLY not triangulated!");
                    }

                    triangles.emplace_back(
                        bvh::Vector3<Scalar>(vPos[face[0]]), 
                        bvh::Vector3<Scalar>(vPos[face[1]]),
                        bvh::Vector3<Scalar>(vPos[face[2]])
                    );
                    auto &norm = triangles.rbegin()->n;
                    normals[face[0]] += norm;
                    normals[face[1]] += norm;
                    normals[face[2]] += norm;        
                }


                for (auto &n : normals) {
                    n = bvh::normalize(n);
                }

                int count = 0;
                for (auto face : fInd) {
                    if (face.size() != 3) {
                        throw std::invalid_argument("PLY not triangulated on SECOND PASS?!");
                    }
                    triangles[count].add_vetex_normals(normals[face[0]], normals[face[1]], normals[face[2]]);
                    triangles[count].set_parent(this);
                    if (color) {
                        triangles[count].add_vertex_colors(vColor[face[0]], vColor[face[1]], vColor[face[2]], 1./255.);
                    }
                    if (tex) {
                        triangles[count].add_vertex_uv(
                            bvh::Vector<float, 2>(texX[face[0]], 1-texY[face[0]]),
                            bvh::Vector<float, 2>(texX[face[1]], 1-texY[face[1]]),
                            bvh::Vector<float, 2>(texX[face[2]], 1-texY[face[2]])
                        );
                    }
                    count++;
                }
            }


        }



        // Entity(const Entity<Scalar> &other) 
        // : triangles(other.triangles), materials(other.materials), material_map(other.material_map) {
        // };

        Entity(const Entity<Scalar>&) = default;
        Entity(Entity<Scalar>&&) = default;

        const std::vector<Triangle> get_triangles() {
            return triangles;
        }

        Material<Scalar> *get_material(float u, float v) {
            return materials[(*material_map)(u, v)].get();
        }

};

#endif