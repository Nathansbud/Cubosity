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
        int hid = -1;
    };

    struct GeomID {
        int VID_MAX = -1;
        int EID_MAX = -1;
        int FID_MAX = -1;
        int HID_MAX = -1;
    };

    struct Vertex {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        HalfEdge* halfEdge;
        Eigen::Vector3f point;
        int vid = -1;
    };

    struct Edge {
        HalfEdge* halfEdge;
        int eid = -1;
    };

    struct Face {
        HalfEdge* halfEdge;
        int fid = -1;
    };

    void fromVerts(const std::vector<Eigen::Vector3f>& vertices, const std::vector<Eigen::Vector3i>& faces, std::unordered_set<HalfEdge*>& halfEdges, GeomID& geom_ids);
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

        // Order: kept ID, removed ID
        std::vector<Vertex*> deletedVertices;

        // Order: collapsed ID, top ID, bottom ID
        std::vector<Edge*> deletedEdges;

        // Order: top ID, bottom ID
        std::vector<Face*> deletedFaces;

        // Wing vert indices for collapse records to re-connect things
        std::pair<int, int> wingVIDs;
    };

    bool collapse(Edge* edge, const Eigen::Vector3f& collapsePoint, CollapseInfo& ci, std::unordered_set<HalfEdge*>& halfEdges);
    bool collapse(HalfEdge* halfEdge, const Eigen::Vector3f& collapsePoint, CollapseInfo& ci, std::unordered_set<HalfEdge*>& halfEdges);

    struct DuplicateInfo {
        std::unordered_map<HalfEdge*, HalfEdge*> oldHalfEdgeToNewHalfEdge;
        std::unordered_map<Vertex*, Vertex*> oldVertToNewVert;
        std::unordered_set<Vertex*> oldVertices;
        std::unordered_set<Face*> newFaces;
    };

    void duplicate(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& newMesh);
    void duplicate(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& newMesh, DuplicateInfo& di);

    void subdivide(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& subdividedMesh);

    void denoise(
        const std::unordered_set<HalfEdge*>& originalMesh,
        std::unordered_set<HalfEdge*>& denoisedMesh,
        const float DIST_THRESH,
        const float SIGMA_C,
        const float SIGMA_S
    );

    Eigen::Matrix4f quadric(const Vertex* vertex);
    void updateError(Edge* edge, const Eigen::Matrix4f& edgeQuadric, std::multimap<float, std::tuple<Edge*, Eigen::Vector3f>>& errorToEdge, std::unordered_map<Edge*, float>& edgeToError);

    // For progressive meshes, we need to be able to walk back the sequence of collapses;
    // hence, we need to get the IDs of all removed elements during the collapse, as well as neighbors
    // that we need to return to their rightful place
    struct CollapseRecord {
        Vertex removedOrigin;
        Vertex shiftedOrigin;

        int collapsedEID;
        int removedEID;
        int shiftedEID;

        int topFID;
        int bottomFID;

        std::pair<int, int> wingVIDs;
    };

    struct CollapseSequence {
        std::vector<CollapseRecord> collapses;
        int initialFaceResolution;
        int finalFaceResolution;
    };

    void simplify(std::unordered_set<HalfEdge*>& originalMesh, const int numTriangles, CollapseSequence& colSeq);

    // Expansion to undo a collapse
    void expand();
};
