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
    // why no work
    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);

    void subdivide();
    void denoise(const float DIST_THRESH, const float SIGMA_C, const float SIGMA_S);
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

    HalfEdge::GeomID _geomID;
};
