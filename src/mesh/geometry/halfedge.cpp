#include "halfedge.h"

#include <QtGlobal>

#include <unordered_set>
#include <optional>
#include <tuple>
#include <iostream>
#include <queue>

using namespace Eigen;

#define SHOULD_VALIDATE 1

void HalfEdge::fromVerts(
    const std::vector<Eigen::Vector3f>& vertices,
    const std::vector<Eigen::Vector3i>& faces,
    std::unordered_set<HalfEdge*>& halfEdges,
    GeomID& geomIDs
) {
    std::vector<Vertex*> verts(vertices.size(), NULL);
    std::map<std::tuple<Vertex*, Vertex*>, Edge*> edges;

    int FID = 0, VID = 0, EID = 0;

    for(; FID < faces.size(); FID++) {
        const Vector3i& face = faces[FID];

        // populate the face half edges
        HalfEdge* faceHalfEdges[3];
        for (int i = 0; i < 3; i++) {
            faceHalfEdges[i] = new HalfEdge();
            halfEdges.insert(faceHalfEdges[i]);
        }

        // initialize the next fields of the half edges.
        for (int i = 0; i < 3; i++) {
            faceHalfEdges[i]->next = faceHalfEdges[(i + 1) % 3]; // 0-1, 1-2, 2-1
        }

        // populate the face vertices
        Vertex* faceVertices[3];
        for (int i = 0; i < 3; i++) {
            int curVertexIndex = face[i];
            Vertex* curVertex;

            // if vertex already exists
            if (verts[curVertexIndex] != NULL) {
                curVertex = verts[curVertexIndex];
            } else {
                // create new vertex
                curVertex = new Vertex();
                curVertex->vid = VID++;
                curVertex->point = vertices[curVertexIndex];
                curVertex->halfEdge = faceHalfEdges[i];

                // save the vertex
                verts[curVertexIndex] = curVertex;
            }

            faceVertices[i] = curVertex;
            faceHalfEdges[i]->vertex = curVertex;
        }

        // populate the face edges
        for (int i = 0; i < 3; i++) {
            Edge* curEdge;

            Vertex* curVertex = faceVertices[i];
            Vertex* nextVertex = faceVertices[(i + 1) % 3];

            // the stored order will always be opposite to the current order
            if (edges.contains({ nextVertex, curVertex })) {
                curEdge = edges.at({ nextVertex, curVertex });

                // if the edge already exists, this half edge should be its twin (and vice-versa)
                faceHalfEdges[i]->twin = curEdge->halfEdge;
                curEdge->halfEdge->twin = faceHalfEdges[i];
            } else {
                curEdge = new Edge();
                curEdge->eid = EID++;
                curEdge->halfEdge = faceHalfEdges[i];

                edges.insert({ { curVertex, nextVertex }, curEdge });
            }

            faceHalfEdges[i]->edge = curEdge;
        }

        // populate the faces
        Face* curFace = new Face();
        curFace->fid = FID;

        curFace->halfEdge = faceHalfEdges[0];
        for (int i = 0; i < 3; i++) {
            faceHalfEdges[i]->face = curFace;
        }
    }

    // Assign our max ID
    geomIDs.EID_MAX = EID;
    geomIDs.FID_MAX = FID;
    geomIDs.VID_MAX = VID;
}

void HalfEdge::toVerts(const std::unordered_set<HalfEdge*>& halfEdges, std::vector<Eigen::Vector3f>& vertices, std::vector<Eigen::Vector3i>& faces) {
    vertices.clear();
    faces.clear();

    std::unordered_set<Face*> foundFaces;
    std::unordered_map<Vertex*, int> vertexToIndex;

    int curIndex = 0;
    for (HalfEdge* halfEdge : halfEdges) {
        if (foundFaces.contains(halfEdge->face)) {
            continue;
        }
        foundFaces.insert(halfEdge->face);

        int vert1, vert2, vert3;
        if (vertexToIndex.contains(halfEdge->vertex)) {
            vert1 = vertexToIndex[halfEdge->vertex];
        } else {
            vert1 = curIndex++;
            vertexToIndex[halfEdge->vertex] = vert1;
            vertices.push_back(halfEdge->vertex->point);
        }

        if (vertexToIndex.contains(halfEdge->next->vertex)) {
            vert2 = vertexToIndex[halfEdge->next->vertex];
        } else {
            vert2 = curIndex++;
            vertexToIndex[halfEdge->next->vertex] = vert2;
            vertices.push_back(halfEdge->next->vertex->point);
        }

        if (vertexToIndex.contains(halfEdge->next->next->vertex)) {
            vert3 = vertexToIndex[halfEdge->next->next->vertex];
        } else {
            vert3 = curIndex++;
            vertexToIndex[halfEdge->next->next->vertex] = vert3;
            vertices.push_back(halfEdge->next->next->vertex->point);
        }

        faces.push_back(Eigen::Vector3i{vert1, vert2, vert3});
    }
}

void HalfEdge::deleteMesh(std::unordered_set<HalfEdge*>& mesh) {
    std::unordered_set<Face*> deletedFaces;
    std::unordered_set<Edge*> deletedEdges;
    std::unordered_set<Vertex*> deletedVertices;
    for (HalfEdge* he : mesh) {
        if (!deletedFaces.contains(he->face)) {
            deletedFaces.insert(he->face);
            delete he->face;
        }

        if (!deletedEdges.contains(he->edge)) {
            deletedEdges.insert(he->edge);
            delete he->edge;
        }

        if (!deletedVertices.contains(he->vertex)) {
            deletedVertices.insert(he->vertex);
            delete he->vertex;
        }

        delete he;
    }

    mesh.clear();
}

void HalfEdge::validate(const std::unordered_set<HalfEdge*>& halfEdges) {
    // If all our mesh operations are implemented correctly, we shouldn't need to validate;
    // hence, make it a no-op if SHOULD_VALIDATE is off
#if(SHOULD_VALIDATE)
    std::unordered_map<Face*, int> faceCount;
    std::unordered_map<Edge*, int> edgeCount;
    std::unordered_map<Vertex*, int> vertexCount;

    for (HalfEdge* halfEdge: halfEdges) {
        // NON-NULL TESTS
        Q_ASSERT(halfEdge->edge != NULL); // no half-edge has a null edge
        Q_ASSERT(halfEdge->edge->halfEdge != NULL); // no edge has a null half-edge

        Q_ASSERT(halfEdge->face != NULL); // no half-edge has a null face
        Q_ASSERT(halfEdge->face->halfEdge != NULL); // no face has a null half-edge

        Q_ASSERT(halfEdge->vertex != NULL); // no half-edge has a null vertex
        Q_ASSERT(halfEdge->vertex->halfEdge != NULL); // no vertex has a null half-edge

        Q_ASSERT(halfEdge->next != NULL); // no half-edge has a null next
        Q_ASSERT(halfEdge->twin != NULL); // every half-edge has a twin

        // NEXT TESTS
        Q_ASSERT(halfEdge->next != halfEdge); // no half-edge points to itself
        Q_ASSERT(halfEdge->next->next->next == halfEdge); // every half edges cycles back to itself

        // TWIN TESTS
        Q_ASSERT(halfEdge->twin != halfEdge); // no half-edges twin is itself
        Q_ASSERT(halfEdge->twin->edge == halfEdge->edge); // twin half-edges share an edge
        Q_ASSERT(halfEdge->twin->face != halfEdge->face); // twin half-edges do not share a face
        Q_ASSERT(halfEdge->twin->vertex != halfEdge->vertex); // twin half-edges do not share an edge
        Q_ASSERT(halfEdge->twin->vertex == halfEdge->next->vertex); // twin half-edges share starting point with next
        Q_ASSERT(halfEdge->twin->twin == halfEdge); // twin's twin points back to yourself

        // FACE TESTS
        Q_ASSERT(halfEdge->face == halfEdge->next->face); // all half-edges nexts share a face
        Q_ASSERT(halfEdge->face->halfEdge->face == halfEdge->face); // a face only reffers to a halfEdge that refers to it.

        // count the number of half-edges on this face
        if (faceCount.contains(halfEdge->face)) {
            faceCount[halfEdge->face] += 1;
        } else {
            faceCount[halfEdge->face] = 1;
        }

        // VERTEX TESTS
        Q_ASSERT(halfEdge->vertex != halfEdge->next->vertex); // next half-edges do not share vertices
        Q_ASSERT(halfEdge->vertex->halfEdge->vertex == halfEdge->vertex); // a vertex only refers to a halfEdge that refers to it.

        // count the number of half-edges on this vertex
        if (vertexCount.contains(halfEdge->vertex)) {
            vertexCount[halfEdge->vertex] += 1;
        } else {
            vertexCount[halfEdge->vertex] = 1;
        }

        // EDGE TESTS
        Q_ASSERT(halfEdge->edge != halfEdge->next->edge); // next half-edges doe not share edges
        Q_ASSERT(halfEdge->edge->halfEdge->edge == halfEdge->edge); // an edge only refers to a halfEdge that referds to it.

        // count the number of half-edges on this edge
        if (edgeCount.contains(halfEdge->edge)) {
            edgeCount[halfEdge->edge] += 1;
        } else {
            edgeCount[halfEdge->edge] = 1;
        }

    }

    for (auto [face, count] : faceCount) {
        Q_ASSERT(count == 3); // all faces should have exactly 3 half-edges
    }

    for (auto [edge, count] : edgeCount) {
        Q_ASSERT(count == 2); // all edges hsould have exactly 2 half-edges
    }

    for (auto [vertex, count] : vertexCount) {
        Q_ASSERT(count >= 3); // all vertices should have at least three half-edges starting at them
    }

    Q_ASSERT(vertexCount.size() + faceCount.size() - 2 == edgeCount.size()); // Euler's characteristic (from Slack)
    Q_ASSERT((float) edgeCount.size() == 1.5 * faceCount.size()); // Zack's response
#endif
}

int HalfEdge::degree(const Vertex* vertex) {
    HalfEdge* start = vertex->halfEdge;
    HalfEdge* curHalfEdge = start;

    int degree = 0;
    do {
        degree += 1;
        curHalfEdge = curHalfEdge->twin->next;
    } while (curHalfEdge != start);

    return degree;
}

float HalfEdge::distance(const Vertex* v1, const Vertex* v2) {
    return (v1->point - v2->point).norm();
}

Eigen::Vector3f HalfEdge::normal(const Vertex* vertex) {
    HalfEdge* start = vertex->halfEdge;
    HalfEdge* cur = start;

    Eigen::Vector3f vertexNormal = Eigen::Vector3f::Zero();
    do {
        vertexNormal += normal(cur->face);
        cur = cur->twin->next;
    } while (cur != start);

    return vertexNormal.normalized();
}

Eigen::Vector3f HalfEdge::normal(const Face* face) {
    Eigen::Vector3f v1 = face->halfEdge->vertex->point;
    Eigen::Vector3f v2 = face->halfEdge->next->vertex->point;
    Eigen::Vector3f v3 = face->halfEdge->next->next->vertex->point;

    return (v2 - v1).cross(v3 - v1).normalized();
}

void HalfEdge::neighbors(const Vertex* vertex, std::unordered_set<HalfEdge*>& outbound, std::unordered_set<Vertex*>& vertices) {
    HalfEdge* start = vertex->halfEdge;
    HalfEdge* cur = start;
    do {
      outbound.insert(cur); // vertex at end of this half-edge
      vertices.insert(cur->next->vertex);
      cur = cur->twin->next;
    } while (cur != start);
}

void HalfEdge::neighborhood(const Vertex* vertex, const std::unordered_set<Vertex*>& vertices, std::unordered_set<Vertex*>& neighbors, float distanceThreshold) {
    std::queue<const Vertex*> unexplored;
    unexplored.push(vertex);

    do {
        const Vertex* curVertex = unexplored.front();
        unexplored.pop();

        HalfEdge* start = curVertex->halfEdge;
        HalfEdge* curHalfEdge = start;

        do {
            Vertex* neighborVertex = curHalfEdge->next->vertex;
            if (neighborVertex != vertex && distance(vertex, neighborVertex) <= distanceThreshold && !neighbors.contains(neighborVertex)) {
                neighbors.insert(neighborVertex);
                unexplored.push(neighborVertex);
            }

            curHalfEdge = curHalfEdge->twin->next;
        } while (curHalfEdge != start);


    } while (!unexplored.empty());
}

int HalfEdge::numSharedVertices(const std::unordered_set<Vertex*>& vertices1, const std::unordered_set<Vertex*>& vertices2) {
    int sharedVerts = 0;
    for (Vertex* vert1: vertices1) {
        if (vertices2.contains(vert1)) {
            sharedVerts++;
        }
    }
    return sharedVerts;
}

bool HalfEdge::flip(Edge* edge) {
    return flip(edge->halfEdge);
}

bool HalfEdge::flip(HalfEdge* halfEdge) {
    // including halfEdge, 6 affected edges
    HalfEdge* prev = halfEdge->next->next;
    HalfEdge* next = halfEdge->next;
    HalfEdge* twin = halfEdge->twin;
    HalfEdge* twinPrev = twin->next->next;
    HalfEdge* twinNext = twin->next;

    // do not flip if degree of either vertex is 3
    if (degree(halfEdge->vertex) == 3 || degree(twin->vertex) == 3) {
        return false;
    }

    // make sure the two vertices do not refer to the flipped edge
    halfEdge->vertex->halfEdge = twinNext;
    twin->vertex->halfEdge = next;

    // make sure the face does not refer to the flipped edge
    halfEdge->face->halfEdge = twinNext;
    twin->face->halfEdge = next;

    // New face halfEdge->prev->twinNext
    halfEdge->next = prev;
    halfEdge->next->next = twinNext;
    halfEdge->next->next->next = halfEdge;
    halfEdge->vertex = twinPrev->vertex;
    twinNext->face = halfEdge->face;

    // New face twin->twinPrev->next
    twin->next = twinPrev;
    twin->next->next = next;
    twin->next->next->next = twin;
    twin->vertex = prev->vertex;
    next->face = twin->face;

    return true;
}

HalfEdge::Vertex* HalfEdge::split(Edge* edge, std::unordered_set<HalfEdge*>& halfEdges) {
    return split(edge->halfEdge, halfEdges);
}

HalfEdge::Vertex* HalfEdge::split(Edge* edge, std::unordered_set<HalfEdge*>& halfEdges, std::unordered_set<Edge*>& newEdges) {
    return split(edge->halfEdge, halfEdges, newEdges);
}

HalfEdge::Vertex* HalfEdge::split(HalfEdge* halfEdge, std::unordered_set<HalfEdge*>& halfEdges) {
    std::unordered_set<Edge*> newEdges;
    return split(halfEdge, halfEdges, newEdges);
}

HalfEdge::Vertex* HalfEdge::split(HalfEdge* halfEdge, std::unordered_set<HalfEdge*>& halfEdges, std::unordered_set<Edge*>& newEdges) {
    // TODO: How tf is this stuff ID'd? This is the only case that isn't handled...

    // including halfEdge, 6 affected edges
    HalfEdge* prev = halfEdge->next->next;
    HalfEdge* next = halfEdge->next;
    HalfEdge* twin = halfEdge->twin;
    HalfEdge* twinPrev = twin->next->next;
    HalfEdge* twinNext = twin->next;

    Eigen::Vector3f midpoint = (halfEdge->vertex->point + twin->vertex->point) / 2.f;
    Vertex* centerVertex = new Vertex();
    centerVertex->point = midpoint; 

    // there are 6 new half edges, starting at the twin of the original halfEdge and moving counter-clockwise
    HalfEdge* he[6];
    for (int i = 0; i < 6; ++i) {
        he[i] = new HalfEdge();
        halfEdges.insert(he[i]);
    }

    // FACE he[0]->twinNext->he[1]
    he[0]->twin = halfEdge;
    he[0]->next = twinNext;
    he[0]->face = twinNext->face;
    he[0]->edge = halfEdge->edge;
    he[0]->edge->halfEdge = he[0]; // make sure th edge points back
    he[0]->vertex = centerVertex;
    centerVertex->halfEdge = he[0]; // new vertex should point to a new edge emenating from it

    twinNext->next = he[1];
    twinNext->face->halfEdge = twinNext;

    he[1]->twin = he[2];
    he[1]->next = he[0];
    he[1]->vertex = twinPrev->vertex;
    he[1]->edge = new Edge();
    he[1]->edge->halfEdge = he[1];
    newEdges.insert(he[1]->edge);
    he[1]->face = twinNext->face;


    // FACE he[2]->twinPrev->twin
    he[2]->twin = he[1];
    he[2]->next = twinPrev;
    he[2]->vertex = centerVertex;
    he[2]->edge = he[1]->edge;
    he[2]->face = new Face();
    he[2]->face->halfEdge = he[2];

    twinPrev->face = he[2]->face;

    twin->next = he[2];
    twin->twin = he[3];
    twin->face = he[2]->face;
    twin->edge = new Edge();
    twin->edge->halfEdge = twin;


    // FACE he[3]->next->he[4]
    he[3]->twin = twin;
    he[3]->next = next;
    he[3]->vertex = centerVertex;
    he[3]->edge = twin->edge;
    he[3]->face = new Face();
    he[3]->face->halfEdge = he[3];

    next->next = he[4];
    next->face = he[3]->face;

    he[4]->twin = he[5];
    he[4]->next = he[3];
    he[4]->vertex = prev->vertex;
    he[4]->edge = new Edge();
    he[4]->edge->halfEdge = he[4];
    newEdges.insert(he[4]->edge);
    he[4]->face = he[3]->face;


    // FACE he[5]->prev->halfEdge
    he[5]->twin = he[4];
    he[5]->next = prev;
    he[5]->vertex = centerVertex;
    he[5]->edge = he[4]->edge;
    he[5]->face = prev->face;
    he[5]->face->halfEdge = he[5]; // make sure the face points back

    halfEdge->twin = he[0];
    halfEdge->next = he[5];

    return centerVertex;
}

bool HalfEdge::causesFlip(const Edge* edge, const Eigen::Vector3f& collapsePoint, Vertex* center, const std::unordered_set<HalfEdge*>& outboundHalfEdges) {
    Eigen::Vector3f oldPos = center->point;
    for (HalfEdge* outbound : outboundHalfEdges) {
        Face* face = outbound->face;
        if (face == edge->halfEdge->face || face == edge->halfEdge->twin->face) continue; // we don't care about the top or bottom faces flipping as they will be deleted

        Eigen::Vector3f curNormal = normal(face);

        center->point = collapsePoint;
        Eigen::Vector3f newNormal = normal(face);

        center->point = oldPos;
        if (curNormal.dot(newNormal) < -0.1) {
//            std::cout << curNormal.dot(newNormal) << std::endl;
            return true;
        }
    }

    return false;
}


bool HalfEdge::canCollapse(const Edge* edge, const Eigen::Vector3f& collapsePoint, CanCollapseInfo& ci) {
    Vertex* left = edge->halfEdge->vertex;
    Vertex* right = edge->halfEdge->twin->vertex;

    neighbors(left, ci.leftOutbound, ci.leftVertices); // all half-edges starting from right verrex
    neighbors(right, ci.rightOutbound, ci.rightVertices); // all half-edges startng from left vertex

    return !causesFlip(edge, collapsePoint, left, ci.leftOutbound)
           && !causesFlip(edge, collapsePoint, right, ci.rightOutbound)
           && numSharedVertices(ci.leftVertices, ci.rightVertices) <= 2;
}

bool HalfEdge::canCollapse(const Edge* edge, const Eigen::Vector3f& collapsePoint) {
    CanCollapseInfo ci;
    return canCollapse(edge, collapsePoint, ci);
}

bool HalfEdge::collapse(Edge* edge, const Eigen::Vector3f& collapsePoint, CollapseInfo& ci, std::unordered_set<HalfEdge*>& halfEdges) {
    return collapse(edge->halfEdge, collapsePoint, ci, halfEdges);
}

bool HalfEdge::collapse(HalfEdge* halfEdge, const Eigen::Vector3f& collapsePoint, CollapseInfo& ci, std::unordered_set<HalfEdge*>& halfEdges) {
    CanCollapseInfo cci;
    if (!canCollapse(halfEdge->edge, collapsePoint, cci)) return false;

    HalfEdge* twin = halfEdge->twin;
    Vertex* left = halfEdge->vertex;
    Vertex* right = halfEdge->twin->vertex;

    cci.leftOutbound.erase(halfEdge);
    cci.rightOutbound.erase(halfEdge->twin);

    Vertex* collapsedVertex = new Vertex();

    // this isn't really a new vertex, so always choose left to be its maker
    collapsedVertex->vid = left->vid;
    collapsedVertex->point = collapsePoint;
    collapsedVertex->halfEdge = halfEdge->next->next->twin;
    ci.collapsedVertex = collapsedVertex;

    // Removed edge wing, shifted edge wing
    ci.wingVIDs = {halfEdge->next->next->vertex->vid, twin->next->next->vertex->vid};

    //TOP FACES
    // delete top face and collapsed edge.
    ci.deletedEdges.push_back(halfEdge->edge);
//    delete halfEdge->edge;
//    delete halfEdge->face;
    ci.deletedFaces.push_back(halfEdge->face);

    // join remaining top 2 faces
    ci.deletedEdges.push_back(halfEdge->next->edge);
//    delete halfEdge->next->edge;
    halfEdge->next->twin->twin = halfEdge->next->next->twin;
    halfEdge->next->next->twin->twin = halfEdge->next->twin;
    halfEdge->next->twin->edge = halfEdge->next->twin->twin->edge;

    halfEdge->next->twin->edge->halfEdge = halfEdge->next->twin; // make sure the edge points back

    halfEdge->next->next->vertex->halfEdge = halfEdge->next->twin;
    halfEdges.erase(halfEdge->next->next);
    delete halfEdge->next->next;

    cci.rightOutbound.erase(halfEdge->next);
    halfEdges.erase(halfEdge->next);
    delete halfEdge->next;

    // BOTTOM FACES
    // delete bottom face
//    delete twin->face;
    ci.deletedFaces.push_back(twin->face);

    // join remaining bottom 2 faces
    ci.deletedEdges.push_back(twin->next->edge);
//    delete twin->next->edge;
    twin->next->twin->twin = twin->next->next->twin;
    twin->next->next->twin->twin = twin->next->twin;
    twin->next->twin->edge = twin->next->twin->twin->edge;

    twin->next->twin->edge->halfEdge = twin->next->twin; // make sure the edge points back

    twin->next->next->vertex->halfEdge = twin->next->twin;
    halfEdges.erase(twin->next->next);
    delete twin->next->next;

    cci.leftOutbound.erase(twin->next);
    halfEdges.erase(twin->next);
    delete twin->next;

    // MOVE VERTICES
    cci.rightVertices.erase(left);
    ci.deletedVertices.push_back(left);
//    delete left;

    for (HalfEdge* outbound : cci.leftOutbound) {
        outbound->vertex = collapsedVertex;
    }

    cci.leftVertices.erase(left);
    ci.deletedVertices.push_back(right);
//    delete right

    for (HalfEdge* outbound : cci.rightOutbound) {
        // All of these neighbors were moved
        ci.movedEdges.insert(outbound->edge->eid);
        outbound->vertex = collapsedVertex;
    }

    // DELETE THE ORGINAL HALF-EDGES
    halfEdges.erase(halfEdge->twin);
    delete halfEdge->twin;

    halfEdges.erase(halfEdge);
    delete halfEdge;
    return true;
}

void HalfEdge::expand(Vertex* collapsed, CollapseRecord& record) {
    // TODO: Any `nullptr` (or similar placeholder field) needs to be figured out!
    // - How to return neighbors?

    // Assume that vertex collapse was obtained by index, record.shiftedOrigin.vid;
    Vertex& shifted = record.shiftedOrigin;
    Vertex& removed = record.removedOrigin;

    if(collapsed->vid != shifted.vid) {
        std::cerr << "Cannot expand vertex " << collapsed->vid << " using collapse record for " << shifted.vid << std::endl;
        return;
    }

    // Reverse the collapsing of edge
    Edge* collapsedEdge = new Edge();
    HalfEdge* collapsedTop = new HalfEdge();
    HalfEdge* collapsedBottom = new HalfEdge();

    // Return our removed edge and all its associated geometry
    Vertex* removedVertex = new Vertex();
    Edge* removedEdge = new Edge();
    HalfEdge* removedInner = new HalfEdge();
    HalfEdge* removedNext = new HalfEdge();

    // Return our shifted edge and all associated geometry
    Edge* shiftedEdge = new Edge();
    HalfEdge* shiftedInner = new HalfEdge();
    HalfEdge* shiftedNext = new HalfEdge();

    Face* topFace = new Face();
    Face* bottomFace = new Face();

    // Remake our collapsed edge!
    collapsedEdge->eid = record.collapsedEID;
    collapsedEdge->halfEdge = collapsedTop;

    collapsedTop->next = removedInner;
    collapsedTop->twin = collapsedBottom;
    collapsedTop->vertex = collapsed;
    collapsedTop->edge = collapsedEdge;
    collapsedTop->face = topFace;
    collapsedTop->hid = -1;

    removedInner->next = removedNext;
    // Indeterminate: removedInner->twin, retrieved from neighbor iteration
    removedInner->vertex = removedVertex;
    removedInner->edge = removedEdge;
    removedInner->face = topFace;
    removedInner->hid = -1;

    removedNext->next = collapsedTop;
    // Indeterminate: removedInner->twin, retrieved from neighbor iteration (outgoing edge top)
    // Indeterminate: removedInner->vertex, retrieved from neighbor iteration (top wing vert)
    // Indeterminate: removedInner->edge, split edge (outgoing edge)
    removedNext->face = topFace;
    removedNext->hid = -1;

    collapsedBottom->next = shiftedInner;
    collapsedBottom->twin = collapsedTop;
    collapsedBottom->vertex = removedVertex;
    collapsedBottom->edge = collapsedEdge;
    collapsedBottom->face = bottomFace;
    collapsedBottom->hid = -1;

    shiftedInner->next = shiftedNext;
    // Indeterminate: shiftedInner->twin, retrieved from neighbor iteration (outgoing)
    shiftedInner->vertex = collapsed;
    shiftedInner->edge = shiftedEdge;
    shiftedInner->face = bottomFace;
    shiftedInner->hid = -1;

    shiftedNext->next = collapsedBottom;
    // Indeterminate: shiftedNext->twin, retrieved from neighbor iteration
    // Indeterminate: shiftedNext->vertex, retrieved from neighbor iteration (bottom wing vert)
    // Indeterminate: shiftedNext->edge, split edge
    shiftedNext->face = bottomFace;
    shiftedNext->hid = -1;

    // Remake our collapsed faces
    topFace->fid = record.topFID;
    topFace->halfEdge = collapsedTop;

    bottomFace->fid = record.bottomFID;
    bottomFace->halfEdge = collapsedBottom;

    removedVertex->point = removed.point;
    removedVertex->halfEdge = removedInner;
    removedVertex->vid = removed.vid;

    // TODO: Neighbor moving!
    HalfEdge* col = collapsed->halfEdge;
    do {
        Vertex* v = col->twin->vertex;

        //            ci.wingVIDs = {halfEdge->next->next->vertex->vid, twin->next->next->vertex->vid};
        HalfEdge* next = col->twin->next;
        if(record.wingVIDs.first == v->vid) {
            // restitch our top face halfedge, non-removed edge
            HalfEdge* priorTwin = col->twin;

            priorTwin->edge = removedEdge;
            priorTwin->twin = removedInner;
            priorTwin->face = topFace;
            priorTwin->next = collapsedTop;

            col->twin = removedNext;

            removedNext->twin = col;
            removedNext->edge = col->edge;

            // ...?
        } else if(record.wingVIDs.second == v->vid) {
            col->twin->face = bottomFace;
            col->twin->next = collapsedBottom;


            // restitch out bottom face halfedge
        }

        // Shmoove all our edges?
        if(record.movedEdges.contains(col->edge->eid)) {

        } else {


        }

        col = next;
    } while(col != collapsed->halfEdge);

    // Return our collapsed vertex to its original position
    collapsed->point = shifted.point;
    collapsed->halfEdge = collapsedTop;
    collapsed->vid = collapsed->vid;

}

void HalfEdge::duplicate(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& newMesh) {
    DuplicateInfo di;
    duplicate(originalMesh, newMesh, di);
}

void HalfEdge::duplicate(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& newMesh, DuplicateInfo& di) {
    std::unordered_map<Edge*, Edge*> oldEdgeToNewEdge;
    std::unordered_map<Face*, Face*> oldFaceToNewFace;

    for (HalfEdge* oldHalfEdge : originalMesh) {
        HalfEdge* newHalfEdge = new HalfEdge();
        newMesh.insert(newHalfEdge);
        newHalfEdge->hid = oldHalfEdge->hid;
        di.oldHalfEdgeToNewHalfEdge.insert({oldHalfEdge, newHalfEdge});
    }

    for (HalfEdge* oldHalfEdge : originalMesh) {
        HalfEdge* newHalfEdge = di.oldHalfEdgeToNewHalfEdge[oldHalfEdge];
        HalfEdge* newTwin = di.oldHalfEdgeToNewHalfEdge[oldHalfEdge->twin];
        HalfEdge* newNext = di.oldHalfEdgeToNewHalfEdge[oldHalfEdge->next];

        newHalfEdge->twin = newTwin;
        newHalfEdge->next = newNext;

        if (oldEdgeToNewEdge.contains(oldHalfEdge->edge)) {
            newHalfEdge->edge = oldEdgeToNewEdge[oldHalfEdge->edge];
        } else {
            Edge* newEdge = new Edge();
            newHalfEdge->edge = newEdge;
            newEdge->halfEdge = newHalfEdge;
            newEdge->eid = oldHalfEdge->edge->eid;
            oldEdgeToNewEdge[oldHalfEdge->edge] = newEdge;
        }

        if (oldFaceToNewFace.contains(oldHalfEdge->face)) {
            newHalfEdge->face = oldFaceToNewFace[oldHalfEdge->face];
        } else {
            Face* newFace = new Face();
            newHalfEdge->face = newFace;
            newFace->halfEdge = newHalfEdge;
            newFace->fid = oldHalfEdge->face->fid;
            oldFaceToNewFace[oldHalfEdge->face] = newFace;
        }

        if (di.oldVertToNewVert.contains(oldHalfEdge->vertex)) {
            newHalfEdge->vertex = di.oldVertToNewVert[oldHalfEdge->vertex];
        } else {
            Vertex* newVertex = new Vertex();
            newHalfEdge->vertex = newVertex;
            newVertex->halfEdge = newHalfEdge;
            newVertex->point = oldHalfEdge->vertex->point;
            newVertex->vid = oldHalfEdge->vertex->vid;
            di.oldVertToNewVert[oldHalfEdge->vertex] = newVertex;
            di.oldVertices.insert(oldHalfEdge->vertex);
        }

    }
}

void HalfEdge::subdivide(const std::unordered_set<HalfEdge*>& originalMesh, std::unordered_set<HalfEdge*>& subdividedMesh) {
    DuplicateInfo di;
    duplicate(originalMesh, subdividedMesh, di);

    std::unordered_set<Edge*> splitEdges;
    std::unordered_set<Edge*> newEdges;
    std::unordered_set<Vertex*> newVertices;

    for (HalfEdge* he : originalMesh) {
        HalfEdge* correspondingHe = di.oldHalfEdgeToNewHalfEdge[he];
        if (splitEdges.contains(he->edge)) continue;

        Vertex* newVert = split(correspondingHe->edge, subdividedMesh, newEdges);
        newVertices.insert(newVert);
        splitEdges.insert(he->edge);
    }

    validate(subdividedMesh);

    for (Edge* e: newEdges) {
        if ((newVertices.contains(e->halfEdge->vertex) && !newVertices.contains(e->halfEdge->twin->vertex)) ||
            (!newVertices.contains(e->halfEdge->vertex) && newVertices.contains(e->halfEdge->twin->vertex))) {
            flip(e);
        }
    }

    for (Vertex* v: newVertices) {
        HalfEdge* cur = v->halfEdge;
        while (newVertices.contains(cur->twin->vertex)) {
            cur = cur->twin->next;
        }

        // cur now ends at v2
        Eigen::Vector3f newPos = (3.f/8.f) * cur->twin->vertex->point;

        // v1 now ends at v1
        HalfEdge* v1 = cur->next->next->twin->next->twin->next;
        newPos += (1.f/8.f) * v1->twin->vertex->point;

        // v4 now ends at v4
        HalfEdge* v4 = v1->next->next->twin->next->twin->next;
        newPos += (3.f/8.f) * v4->twin->vertex->point;

        // v3 now ends at v3
        HalfEdge* v3 = v4->next->twin->next->next->twin->next->twin->next;
        newPos += (1.f/8.f) * v3->twin->vertex->point;

        v->point = newPos;
    }

    for (auto const& [oldVert, duplicatedVert] : di.oldVertToNewVert) {
        int n = degree(oldVert);

        float u;
        if (n == 3) {
            u = 3.f/16.f;
        } else {
            u = (1.f / n) * ((5.f/8.f) - powf((3.f/8.f) + (1.f/4.f)*cosf((2.f * M_PI) / n), 2));
        }


        Eigen::Vector3f newPos = (1.f - n*u) * oldVert->point;

        HalfEdge* start = oldVert->halfEdge;
        HalfEdge* cur = start;

        do {
          newPos += cur->twin->vertex->point * u;
          cur = cur->twin->next;
        } while (cur != start);

        duplicatedVert->point = newPos;
    }

    validate(subdividedMesh);
}

void HalfEdge::denoise(
    const std::unordered_set<HalfEdge*>& originalMesh,
    std::unordered_set<HalfEdge*>& denoisedMesh,
    const float DIST_THRESH,
    const float SIGMA_C,
    const float SIGMA_S
) {
    DuplicateInfo di;
    duplicate(originalMesh, denoisedMesh, di);

    for (auto const& [oldVert, duplicatedVert] : di.oldVertToNewVert) {
        std::unordered_set<Vertex*> neighbors;
        neighborhood(oldVert, di.oldVertices, neighbors, DIST_THRESH);

        Eigen::Vector3f oldVertexNormal = normal(oldVert);

        float sum = 0;
        float normalizer = 0;
        for (Vertex* neighbor : neighbors) {

            float t = (oldVert->point - neighbor->point).norm();
            float h = oldVertexNormal.dot(neighbor->point - oldVert->point);

            float wc = exp(-powf(t, 2) / (2 * powf(SIGMA_C, 2)));
            float ws = exp(-powf(h, 2) / (2 * powf(SIGMA_S, 2)));

            sum += (wc * ws) * h;
            normalizer += (wc * ws);
        }

        if (neighbors.size() > 0) {
            duplicatedVert->point += oldVertexNormal * (sum / normalizer);
        }
    }

}

Eigen::Matrix4f HalfEdge::quadric(const Vertex* vertex) {
    Eigen::Matrix4f q = Eigen::Matrix4f::Zero();

    HalfEdge* start = vertex->halfEdge;
    HalfEdge* cur = start;

    do {
        Eigen::Vector3f faceNorm = normal(cur->face);
        Eigen::Vector4f plane = Eigen::Vector4f(faceNorm.x(), faceNorm.y(), faceNorm.z(), -vertex->point.dot(faceNorm));
        q += plane * plane.transpose();

        cur = cur->twin->next;
    } while (cur != start);

    return q;
}

void HalfEdge::updateError(Edge* edge, const Eigen::Matrix4f& edgeQuadric, std::multimap<float, std::tuple<Edge*, Eigen::Vector3f>>& errorToEdge, std::unordered_map<Edge*, float>& edgeToError) {
    Eigen::Matrix4f errorQuadric;
    errorQuadric << edgeQuadric.coeff(0, 0), edgeQuadric.coeff(0, 1), edgeQuadric.coeff(0, 2), edgeQuadric.coeff(0, 3),
                    edgeQuadric.coeff(0, 1), edgeQuadric.coeff(1, 1), edgeQuadric.coeff(1, 2), edgeQuadric.coeff(1, 3),
                    edgeQuadric.coeff(0, 2), edgeQuadric.coeff(1, 2), edgeQuadric.coeff(2, 2), edgeQuadric.coeff(2, 3),
                    0.f,                     0.f,                     0.f,                     1.f;

    bool invertible;
    Eigen::Matrix4f inverse;
    errorQuadric.computeInverseWithCheck(inverse, invertible);

    Eigen::Vector3f collapsePoint;
    Eigen::Vector4f homogenousCollapsePoint;
    if (invertible) {
        homogenousCollapsePoint = inverse * Eigen::Vector4f(0.f, 0.f, 0.f, 1.f);
        collapsePoint << homogenousCollapsePoint.x(), homogenousCollapsePoint.y(), homogenousCollapsePoint.z();
    } else {
        collapsePoint = ((edge->halfEdge->vertex->point + edge->halfEdge->twin->vertex->point) / 2.f);
        homogenousCollapsePoint << collapsePoint.x(), collapsePoint.y(), collapsePoint.z(), 1.f;
    }

    if (!canCollapse(edge, collapsePoint)) {
        errorToEdge.insert({ std::numeric_limits<float>::infinity(), { edge, Eigen::Vector3f::Zero() } }); // un-collapsable edges should be inifinite weight
        edgeToError[edge] = std::numeric_limits<float>::infinity();
    } else {
        float error = homogenousCollapsePoint.transpose() * edgeQuadric * homogenousCollapsePoint;
        errorToEdge.insert({ error, { edge, collapsePoint } });
        edgeToError[edge] = error;
    }
}

void HalfEdge::simplify(
    std::unordered_set<HalfEdge*>& mesh,
    const int numTriangles,
    CollapseSequence& colSeq
) {
    std::unordered_map<Vertex*, Eigen::Matrix4f> vertToQuadric;
    std::unordered_set<Face*> faces;

    std::multimap<float, std::tuple<Edge*, Eigen::Vector3f>> errorToEdge;
    std::unordered_map<Edge*, float> edgeToError;

    for (HalfEdge* he : mesh) {
        if (!faces.contains(he->face)) {
            faces.insert(he->face);
        }

        Vertex* v1 = he->vertex;
        Vertex* v2 = he->twin->vertex;
        Edge* curEdge = he->edge;

        if (edgeToError.contains(curEdge)) continue;

        Eigen::Matrix4f q1;
        if (vertToQuadric.contains(v1)) {
            q1 = vertToQuadric[v1];
        } else {
            q1 = quadric(v1);
            vertToQuadric[v1] = q1;
        }

        Eigen::Matrix4f q2;
        if (vertToQuadric.contains(v2)) {
            q2 = vertToQuadric[v2];
        } else {
            q2 = quadric(v2);
            vertToQuadric[v2] = q2;
        }

        Eigen::Matrix4f edgeQuadric = (q1 + q2);
        updateError(curEdge, edgeQuadric, errorToEdge, edgeToError);
    }

    int curTriangles = faces.size();
    colSeq.initialFaceResolution = curTriangles;

    while (curTriangles > numTriangles) {
        auto [error, edgeAndPoint] = *errorToEdge.begin();
        auto [edge, collapsePoint] = edgeAndPoint;

        CollapseInfo ci;
        if(!collapse(edge, collapsePoint, ci, mesh)) {
            break;
        }

        CollapseRecord cr;

        // Pass through our wing vertices and moved edges for expansion, as we need
        // to return all our original neighbors
        cr.wingVIDs = ci.wingVIDs;
        cr.movedEdges = ci.movedEdges;

        // Explicitly perform our geometry deletions here, since we didn't delete them within the function

        // Populate our shifted and removed vertices
        Vertex* shiftedVert = ci.deletedVertices[0];
        cr.shiftedOrigin = { .point = shiftedVert->point, .vid = shiftedVert->vid };

        Vertex* deletedVert = ci.deletedVertices[1];
        cr.removedOrigin = { .point = deletedVert->point, .vid = deletedVert->vid };

        delete shiftedVert;
        delete deletedVert;

        // Populate our removed edges; our collapsed edge can be explicitly recreated,
        // the other two will need to be stitched together using our removed edge
        Edge* collapsedEdge = ci.deletedEdges[0];
        Edge* removedEdge = ci.deletedEdges[2];
        Edge* shiftedEdge = ci.deletedEdges[1];

        // Collapsed edge is between to be shifted & removed vertices
        cr.collapsedEID = collapsedEdge->eid;
        cr.removedEID = removedEdge->eid;
        cr.shiftedEID = shiftedEdge->eid;

        delete collapsedEdge;
        delete shiftedEdge;
        delete removedEdge;

        Face* topFace = ci.deletedFaces[0];
        Face* bottomFace = ci.deletedFaces[1];

        cr.topFID = topFace->fid;
        cr.bottomFID = bottomFace->fid;

        delete topFace;
        delete bottomFace;

        validate(mesh);

        Vertex* collapsedVertex = ci.collapsedVertex;
        for (Edge* deletedEdge : ci.deletedEdges) {
            float edgeError = edgeToError[deletedEdge];

            auto it = errorToEdge.find(edgeError);
            while (get<0>(it->second) != deletedEdge) it++; // if multiple share the same error make sure you are finding the actual edge
            errorToEdge.erase(it);

            edgeToError.erase(deletedEdge);
        }

        for (Vertex* deletedVertex : ci.deletedVertices) {
            vertToQuadric.erase(deletedVertex);
        }

        // add quadric of new vertex
        vertToQuadric[collapsedVertex] = quadric(collapsedVertex);

        std::unordered_set<Vertex*> updatedVertices;
        updatedVertices.insert(collapsedVertex);

        std::unordered_set<Edge*> updatedEdges;

        HalfEdge* start = collapsedVertex->halfEdge;
        HalfEdge* cur = start;
        do {
            Edge* curEdge = cur->edge;
            if (updatedEdges.contains(curEdge)) {
                cur = cur->twin->next;
                continue;
            }

            updatedEdges.insert(curEdge);

            Vertex* endVertex = cur->twin->vertex;
            if (!updatedVertices.contains(endVertex)) {
                vertToQuadric[endVertex] = quadric(endVertex); // re-calculate quadric for neighbor verts
                updatedVertices.insert(endVertex);
            }

            float oldEdgeError = edgeToError[curEdge];
            auto it = errorToEdge.find(oldEdgeError);
            while (get<0>(it->second) != curEdge) it++; // if multiple share the same error make sure you are finding the actual edge
            errorToEdge.erase(it); // get rid of it

            // calculate new error of edge
            Eigen::Matrix4f q1 = vertToQuadric[collapsedVertex];
            Eigen::Matrix4f q2 = vertToQuadric[endVertex];
            Eigen::Matrix4f edgeQuadric = (q1 + q2);
            updateError(curEdge, edgeQuadric, errorToEdge, edgeToError);

            HalfEdge* secondRingStart = endVertex->halfEdge;
            HalfEdge* secondRingCur = secondRingStart;
            do {
                Edge* curSecondRingEdge = secondRingCur->edge;

                if (updatedEdges.contains(curSecondRingEdge)) {
                    secondRingCur = secondRingCur->twin->next;
                    continue;
                }
                updatedEdges.insert(curSecondRingEdge);

                Vertex* endSecondRingVertex = secondRingCur->twin->vertex;
                if (!updatedVertices.contains(endSecondRingVertex)) {
                    vertToQuadric[endSecondRingVertex] = quadric(endSecondRingVertex); // re-calculate quadric for neighbor verts
                    updatedVertices.insert(endSecondRingVertex);
                }

                float oldEdgeError = edgeToError[curSecondRingEdge];
                auto it = errorToEdge.find(oldEdgeError);
                while (get<0>(it->second) != curSecondRingEdge) it++; // if multiple share the same error make sure you are finding the actual edge
                errorToEdge.erase(it); // get rid of it

                // calculate new error of edge
                Eigen::Matrix4f q1 = vertToQuadric[endVertex];
                Eigen::Matrix4f q2 = vertToQuadric[endSecondRingVertex];
                Eigen::Matrix4f edgeQuadric = (q1 + q2);
                updateError(curSecondRingEdge, edgeQuadric, errorToEdge, edgeToError);

                secondRingCur = secondRingCur->twin->next;
            } while (secondRingCur != secondRingStart);

            cur = cur->twin->next;
        } while (cur != start);

        colSeq.collapses.push_back(cr);
        curTriangles -= 2;
    }

    colSeq.finalFaceResolution = curTriangles;
}
