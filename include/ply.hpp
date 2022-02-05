#ifndef PLY_HPP
#define PLY_HPP

#include <vector>
#include <string>
#include <optional>

#include "bvh/triangle.hpp"
#include "bvh/vector.hpp"


#include "happly.hpp"

namespace ply {

template <typename Scalar>
inline std::vector<bvh::Triangle<Scalar> > load_from_file(const std::string& file) {
    
    // Construct the data object by reading from file
    happly::PLYData plyIn(file);

    // Get mesh-style data from the object
    auto vPos = plyIn.getVertexPositions();
    auto fInd = plyIn.getFaceIndices<size_t>();
    auto vColor = plyIn.getVertexColors();

    std::vector<bvh::Triangle<Scalar> > triangles;
    std::vector<bvh::Vector3<Scalar> > normals(vPos.size(), bvh::Vector3<Scalar>(0,0,0));    

    for (auto face : fInd) {
        if (face.size() != 3) {
            std::cerr << "PLY not triangulated!" << std::endl; 
            return std::vector<bvh::Triangle<Scalar> >();
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
            std::cerr << "PLY not triangulated on SECOND PASS???" << std::endl; 
            return std::vector<bvh::Triangle<Scalar> >();
        }
        triangles[count].add_vetex_normals(normals[face[0]], normals[face[1]], normals[face[2]]);
        if (!vColor.empty()) {
            std::cout << "Applying colors..." << std::endl;
            triangles[count].add_vertex_colors(vColor[face[0]], vColor[face[1]], vColor[face[2]], 1./255.);
        }
        count++;
    }
    std::cout << "Loaded " << count << " triangles from " << file << std::endl;
    return triangles;
}

} // namespace ply

#endif