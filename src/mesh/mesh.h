#pragma once

#include <vector>

#include "Eigen/StdVector"

#include "geometry/halfedge.h"

class Mesh
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices,
                         const std::vector<Eigen::Vector3i> &faces);

    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);

    void subdivide();
    void denoise();
    void simplify(const int n);

    const std::vector<Eigen::Vector3f>& getVertices() {
        return _vertices;
    }
    const std::vector<Eigen::Vector3i>& getFaces() {
        return _faces;
    }

private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
    std::unordered_set<HalfEdge::HalfEdge*> _halfEdges;
};
