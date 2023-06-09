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

    bool loadProgressiveMesh(const std::string &stampPath,
                             const std::vector<Eigen::Vector3f> &vertices,
                             const std::vector<Eigen::Vector3i> &faces);

    void saveMesh(const std::string &outputPath);
    void saveProgressiveMesh(const std::string &outputDir);

    void subdivide();
    void denoise(const float DIST_THRESH, const float SIGMA_C, const float SIGMA_S);
    void simplify(const int n, std::string& outputDir);
    bool expand(int toLevel);

    const std::vector<Eigen::Vector3f>& getVertices() { return _vertices; }
    const std::vector<Eigen::Vector3i>& getFaces() { return _faces; }

    void update() { HalfEdge::toVerts(_halfEdges, _vertices, _faces, _indices); }

    int getOrientationGroup(int vIndex) {
        return _indices.vidToProperties[_indices.vindexToVIDs[vIndex]].orientationGroup;
    }
    void setOrientationGroup(int vIndex, int groupID) {
        _indices.vidToProperties[_indices.vindexToVIDs[vIndex]].orientationGroup = groupID;
    }

    void updatePositions(const std::vector<Eigen::Vector3f> &vertices);
private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
    std::unordered_set<HalfEdge::HalfEdge*> _halfEdges;

    // Mapping from OpenGL ordering to geometry element ID
    HalfEdge::GeomMap _geometry;
    HalfEdge::IndexMap _indices;
    HalfEdge::CollapseState _collapseState;

    bool loadedProgressive = false;
};
