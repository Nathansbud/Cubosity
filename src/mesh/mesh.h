#pragma once

#include <vector>
#include <queue>
#include <set>
#include <map>
#include <unordered_set>
#include <unordered_map>
#include <limits>

#include "Eigen/StdVector"
#include "./geometry/halfedge.h"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f);
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i);

enum class SplitMode { LOOP = 0, MIDPOINT = 1 };
enum class CollapseMode { LEFT = 0, RIGHT = 1, MIDPOINT = 1 };

class Mesh {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    // Default constructor
    Mesh() {}

    // Clone constructor (not a valid C++ copy constructor!)
    Mesh(Mesh& old, std::map<Halfedge*, Halfedge*>& remapper);

    ~Mesh();

    void createHalfedge(const std::vector<Vector3f>& verts, const std::vector<Vector3i>& faces);
    void createHalfedge();

    Halfedge* cloneHalfedge();

    bool valid();
    bool canCollapse(Edge* e);

    // ATOMIC OPERATIONS //
    bool edgeFlip(Edge* originalEdge);
    std::vector<Edge*> edgeSplit(Edge* originalEdge, SplitMode mode);
    std::set<Edge*> edgeCollapse(Edge* originalEdge, Vector3f collapseTo);

    // MESH OPERATIONS //
    void noisify(float weight);
    void loopSubdivide(int iterations);
    void isotropicRemesh(int iterations, float weight);
    void bilaterialDenoise(int iterations, float sigC, float sigS, float kernelWidth);

    // Quadric simplification and associated helper functions
    void updateFaceQuadric(Face* f, std::unordered_map<Face*, Matrix4f>& face_quadrics);
    void updateVertexQuadric(Vertex* v, std::unordered_map<Vertex*, Matrix4f>& vertex_quadrics, std::unordered_map<Face*, Matrix4f>& face_quadrics);
    void updateEdgeError(Edge* edge, std::multimap<float, std::pair<Edge*, Vector3f>>& candidates, std::unordered_map<Vertex*, Matrix4f>& vertex_quadrics);
    void simplifyQuadric(int targetFaces);

    // MESH ELEMENTS //
    std::unordered_set<Vertex*> vertices;
    std::unordered_set<Face*> faces;
    std::unordered_set<Halfedge*> halfedges;
    std::unordered_set<Edge*> edges;

    // INITIAL LOAD //
    void initFromVectors(const std::vector<Eigen::Vector3f> &vertices, const std::vector<Eigen::Vector3i> &faces);
    void loadFromFile(const std::string &filePath);
    void saveToFile(const std::string &filePath);
private:
    std::vector<Eigen::Vector3f> _vertices;
    std::vector<Eigen::Vector3i> _faces;
};
