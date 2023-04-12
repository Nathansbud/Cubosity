#pragma once

#include <vector>
#include <Eigen/Dense>
#include <unordered_set>
#include <unordered_map>
#include <map>

namespace HalfEdge {
    struct HalfEdge;
    struct Vertex;
    struct Edge;
    struct Face;

    struct HalfEdge {
        HalfEdge* twin;
        HalfEdge* next;

        Vertex* vertex;
        Edge* edge;
        Face* face;
    };

    struct Vertex {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        HalfEdge* halfEdge;
        Eigen::Vector3f point;
    };

    struct Edge {
        HalfEdge* halfEdge;
    };

    struct Face {
        HalfEdge* halfEdge;
    };

    void fromVerts(const std::vector<Eigen::Vector3f>& vertices, const std::vector<Eigen::Vector3i>& faces, std::unordered_set<HalfEdge*>& halfEdges);
    void toVerts(const std::unordered_set<HalfEdge*>& halfEdges, std::vector<Eigen::Vector3f>& vertices, std::vector<Eigen::Vector3i>& faces);
    void deleteMesh(std::unordered_set<HalfEdge*>& mesh);
    void validate(const std::unordered_set<HalfEdge*>& halfEdges);

    int degree(const Vertex* vertex);
    float distance(const Vertex* v1, const Vertex* v2);
    Eigen::Vector3f normal(const Vertex* vertex);
    Eigen::Vector3f normal(const Face* face);
    void neighbors(const Vertex* vertex, std::unordered_set<HalfEdge*>& outbound, std::unordered_set<Vertex*>& vertices);
    void neighborhood(const Vertex* vertex, const std::unordered_set<Vertex*>& vertices, std::unordered_set<Vertex*>& neighbors, float distance);
    int numSharedVertices(const std::unordered_set<Vertex*>& vertices1, const std::unordered_set<Vertex*>& vertices2);

    bool flip(HalfEdge* halfEdge);
    bool flip(Edge* edge);

    Vertex* split(Edge* edge, std::unordered_set<HalfEdge*>& halfEdges);
    Vertex* split(Edge* edge, std::unordered_set<HalfEdge*>& halfEdges, std::unordered_set<Edge*>& newEdges);
    Vertex* split(HalfEdge* halfEdge, std::unordered_set<HalfEdge*>& halfEdges);
    Vertex* split(HalfEdge* halfEdge, std::unordered_set<HalfEdge*>& halfEdges, std::unordered_set<Edge*>& newEdges);

    struct CanCollapseInfo {
        std::unordered_set<HalfEdge*> rightOutbound;
        std::unordered_set<Vertex*> rightVertices;
        std::unordered_set<HalfEdge*> leftOutbound;
        std::unordered_set<Vertex*> leftVertices;
    };

    bool causesFlip(const Edge* edge, const Eigen::Vector3f& collapsePoint, Vertex* center, const std::unordered_set<HalfEdge*>& outboundHalfEdges);
    bool canCollapse(const Edge* edge, const Eigen::Vector3f& collapsePoint, CanCollapseInfo& ci);
    bool canCollapse(const Edge* edge, const Eigen::Vector3f& collapsePoint);

    struct CollapseInfo {
        Vertex* collapsedVertex;
        std::unordered_set<Edge*> deletedEdges;
        std::unordered_set<Vertex*> deletedVertices;
    };

    bool collapse(Edge* edge, const Eigen::Vector3f& collapsePoint, CollapseInfo& ci, std::unordered_set<HalfEdge*>& halfEdges);
    bool collapse(HalfEdge* halfEdge, const Eigen::Vector3f& collapsePoiht, CollapseInfo& ci, std::unordered_set<HalfEdge*>& halfEdges);

    struct DuplicateInfo {
        std::unordered_map<HalfEdge*, HalfEdge*> oldHalfEdgeToNewHalfEdge;
        std::unordered_map<Vertex*, Vertex*> oldVertToNewVert;
        std::unordered_set<Vertex*> oldVertices;
        std::unordered_set<Face*> newFaces;
    };

    void duplicate(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& newMesh);
    void duplicate(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& newMesh, DuplicateInfo& di);

    void subdivide(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& subdividedMesh);

    void denoise(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& denoisedMesh);

    Eigen::Matrix4f quadric(const Vertex* vertex);
    void updateError(Edge* edge, const Eigen::Matrix4f& edgeQuadric, std::multimap<float, std::tuple<Edge*, Eigen::Vector3f>>& errorToEdge, std::unordered_map<Edge*, float>& edgeToError);
    void simplify(std::unordered_set<HalfEdge*>& originalMesh, const int numTriangles);
};
