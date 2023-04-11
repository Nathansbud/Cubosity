#include "mesh.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>

//#define TINYOBJLOADER_IMPLEMENTATION
#include "util/tiny_obj_loader.h"
#include "util/zutils.h"

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const vector<Vector3f> &vertices, const vector<Vector3i> &faces) {
    // Copy vertices and faces into internal vector
    _vertices = vertices;
    _faces    = faces;
}

bool Mesh::valid() {
    if(!(this->faces.size() + this->vertices.size() - 2 == this->edges.size())) {
        std::cout << "Failed Euler characteristic ["
                  << this->faces.size() << "F, "
                  << this->vertices.size() << "V, "
                  << this->edges.size() << "E";
        return false;
    }

    for(const auto& hedge : this->halfedges) {
        std::vector<bool> conds = {
            /* 0 */ hedge->nonNull(),
            /* 1 */ hedge->twin && hedge->twin != hedge && hedge->twin->twin && hedge->twin->twin == hedge,
            /* 2 */ hedge->next && hedge->next->next && hedge->next->next->next && hedge->next->next->next == hedge,
            /* 3 */ hedge->twin->next->next->next->twin->next->next->next == hedge,
            /* 4 */ hedge->edge == hedge->twin->edge,
            /* 5 */ hedge->edge != hedge->next->edge && hedge->edge != hedge->next->next->edge,
            /* 6 */ hedge->face == hedge->next->face && hedge->face == hedge->next->next->face,
            /* 7 */ hedge->face != hedge->twin->face && hedge->next->face != hedge->next->twin->face && hedge->next->next->face != hedge->next->next->twin->face,
            /* 8 */ hedge->twin && hedge->twin->vertex != hedge->vertex
        };

        for(int c = 0; c < conds.size(); c++) {
            if(!conds[c]) {
                std::cout << "Condition " << c << " not met on hedge " << hedge << std::endl;
                return false;
            }
        }
    }

    return true;
}

bool Mesh::canCollapse(Edge* e) {
    Halfedge* root = e->halfedge;
    Halfedge* twin = root->twin;

    Halfedge* curr = twin->next;
    std::set<Vertex*> neighbors;
    do {
        neighbors.insert(curr->twin->vertex);
        curr = curr->twin->next;
    } while(curr != root);

    int shared = 0;
    curr = root->next;
    do {
        if(neighbors.contains(curr->twin->vertex)) ++shared;
        curr = curr->twin->next;
    } while(curr != twin);

    // We can only edge collapse if we have exactly 2 neighbors;
    // allowing <2 means the mesh can degenerate :(
    return shared == 2;
}

std::set<Edge*> Mesh::edgeCollapse(Edge* edge, Vector3f collapse) {
    // If we cannot collapse our edge, return it
    if(!this->canCollapse(edge)) {
        return {};
    }

    /* We have twin faces joined by an edge which we want to delete:
     *
     *        A
     *        o        ...
     *      /  \     /
     *    /  R  \  /
     * B o-------o D
     *   \  T  /  \
     *    \  /     \
     *     o        ...
     *     C
     *
     * Our goal is going to be: delete AB, BC, BD, R, T, and all interior halfedges,
     * as well as delete D and reattach all its halfedges to B (but this should not require next changes)
     */

    Halfedge* root = edge->halfedge;
    Halfedge* rootNext = root->next;
    Halfedge* rootLast = rootNext->next;

    Halfedge* twin = root->twin;
    Halfedge* twinNext = twin->next;
    Halfedge* twinLast = twinNext->next;

    Halfedge* rootNextTwin = rootNext->twin;
    Halfedge* rootLastTwin = rootLast->twin;

    Halfedge* twinNextTwin = twinNext->twin;
    Halfedge* twinLastTwin = twinLast->twin;

    Vertex* A = rootLast->vertex;
    Vertex* B = root->vertex;
    Vertex* C = twinLast->vertex;
    Vertex* D = twin->vertex;

    Edge* AB = rootLast->edge;
    Edge* BC = twinNext->edge;

    Face* R = root->face;
    Face* T = twin->face;

    Halfedge* curr = twin;

    // Normal flip heuristic!
    do {
        Face* f = curr->face;
        Vertex* v = curr->vertex;
        if(f != R && f != T) {
            Halfedge* fh = f->halfedge;
            while(fh->vertex != v) fh = fh->next;

            Vector3f oldNormal = (fh->next->vertex->position - v->position).cross(
                fh->next->next->vertex->position - v->position
            ).normalized();

            Vector3f newNormal = (fh->next->vertex->position - collapse).cross(
                fh->next->next->vertex->position - collapse
            ).normalized();

            if(oldNormal.dot(newNormal) < -0.5) {
                std::cout << "Refusing flip; normal dot product was " << oldNormal.dot(newNormal) << std::endl;
                return {};
            }
        }

        curr = curr->twin->next;
    } while(curr != twin);

    do {
        curr->vertex = B;
        curr = curr->twin->next;
    } while(curr != twin);


    rootLastTwin->twin = rootNextTwin;
    rootNextTwin->twin = rootLastTwin;

    rootLastTwin->edge = rootNext->edge;
    rootLastTwin->edge->halfedge = rootNextTwin;

    twinNextTwin->twin = twinLastTwin;
    twinLastTwin->twin = twinNextTwin;

    twinNextTwin->edge = twinLastTwin->edge;
    twinLastTwin->edge->halfedge = twinNextTwin;

    A->halfedge = rootNextTwin;
    B->halfedge = twinLastTwin;
    B->position = collapse;
    C->halfedge = twinNextTwin;

    for(const auto& hedge : {root, rootNext, rootLast, twin, twinNext, twinLast}) {
        delete hedge;
        this->halfedges.erase(hedge);
    }

    for(const auto& edge : {edge, AB, BC}) {
        delete edge;
        this->edges.erase(edge);
    }

    for(const auto& face : {R, T}) {
        delete face;
        this->faces.erase(face);
    }

    delete D;
    this->vertices.erase(D);

    return {edge, AB, BC};
}

bool Mesh::edgeFlip(Edge* originalEdge) {
    Halfedge* top = originalEdge->halfedge;
    Halfedge* topNext = top->next;
    Halfedge* topLast = top->next->next;

    Halfedge* bottom = top->twin;
    Halfedge* bottomNext = bottom->next;
    Halfedge* bottomLast = bottom->next->next;

    int topDeg = 0;
    Halfedge* t = top;
    do {
        ++topDeg;
        t = t->twin->next;
    } while(t != top);

    // We only want to apply an edge flip if degree > 3
    if(topDeg <= 3) return false;

    int botDeg = 0;
    Halfedge* b = bottom;
    do {
        ++botDeg;
        b = b->twin->next;
    } while(b != bottom);

    if(botDeg <= 3) return false;

    Vertex* v2 = top->vertex;
    Vertex* v4 = bottom->vertex;

    Vertex* v1 = topLast->vertex;
    Vertex* v3 = bottomLast->vertex;

    v2->halfedge = bottomNext;
    top->face->halfedge = bottomNext;

    v4->halfedge = topNext;
    bottom->face->halfedge = topNext;

    top->vertex = v3;

    bottomNext->next = top;
    top->next = topLast;
    topLast->next = bottomNext;
    bottomNext->face = top->face;

    bottom->vertex = v1;

    topNext->next = bottom;
    bottom->next = bottomLast;
    bottomLast->next = topNext;
    topNext->face = bottom->face;

    return true;
}

std::vector<Edge*> Mesh::edgeSplit(Edge* originalEdge, SplitMode mode) {
    // We want to avoid adding our new edges directly on account of the fact that we may implement this algorithm in a global context,
    // In which case we would invalidate our edge iterator; this may be (no, is) a bad design decision, but oh well
    std::vector<Edge*> newEdges;
    newEdges.reserve(3);

    Halfedge* root = originalEdge->halfedge;

    Halfedge* cNext = root->next;
    Halfedge* cLast = root->next->next;

    Halfedge* twin = root->twin;
    Halfedge* twinNext = twin->next;
    Halfedge* twinLast = twin->next->next;

    Vertex* v2 = root->vertex;
    Vertex* v4 = cNext->vertex;
    Vertex* v1 = cLast->vertex;
    Vertex* v3 = twinLast->vertex;

    // bottom left face remains what it previously was
    Halfedge* bottomLeft = new Halfedge();
    Edge* leftBar = new Edge(bottomLeft);

    Vertex* nv = new Vertex(
        (v2->position + v4->position) / 2.,
        bottomLeft
    );

    nv->flag = mode == SplitMode::LOOP;

    root->next = bottomLeft;
    root->face->halfedge = root;
    root->edge->halfedge = root;

    bottomLeft->vertex = nv;
    bottomLeft->next = cLast;
    bottomLeft->edge = leftBar;
    bottomLeft->face = root->face;

    cLast->next = root;

    // We split our initial edge in two, so added a new upper half, which has a new edge!
    Halfedge* upperHalf = new Halfedge();
    Edge* splitEdge = new Edge(upperHalf);

    Halfedge* upperLeft = new Halfedge();
    Face* topLeftFace = new Face(upperLeft);

    upperHalf->vertex = nv;
    upperHalf->next = cNext;
    upperHalf->edge = splitEdge;
    upperHalf->face = topLeftFace;

    cNext->next = upperLeft;
    cNext->face = topLeftFace;

    upperLeft->vertex = v1;
    upperLeft->next = upperHalf;
    upperLeft->edge = leftBar;
    upperLeft->face = topLeftFace;

    upperLeft->twin = bottomLeft;
    bottomLeft->twin = upperLeft;

    // Root last next is already root

    // Take it back now y'all...gotta run the same logic on the twin direction

    upperHalf->twin = twin;
    twin->twin = upperHalf;
    twin->edge = splitEdge;

    //twin face stays the same, but make sure our face points to it
    twin->face->halfedge = twinLast;

    Halfedge* topRight = new Halfedge();
    Edge* rightBar = new Edge(topRight);

    twin->next = topRight;

    topRight->vertex = nv;
    topRight->next = twinLast;
    topRight->edge = rightBar;
    topRight->face = twin->face;

    // Twin last next is already twin

    Halfedge* lowerHalf = new Halfedge();
    Face* bottomRightFace = new Face(lowerHalf);

    root->twin = lowerHalf;
    lowerHalf->twin = root;

    lowerHalf->vertex = nv;
    lowerHalf->next = twinNext;
    lowerHalf->edge = originalEdge;
    lowerHalf->face = bottomRightFace;

    Halfedge* bottomRight = new Halfedge();

    twinNext->face = bottomRightFace;
    twinNext->next = bottomRight;
    topRight->twin = bottomRight;

    bottomRight->twin = topRight;
    bottomRight->vertex = v3;
    bottomRight->next = lowerHalf;
    bottomRight->edge = rightBar;
    bottomRight->face = bottomRightFace;

    // We created:

    // 1 vertex
    this->vertices.insert(nv);

    // 2 new faces (reused our original 2 as our bottom left, top right)...
    this->faces.insert({topLeftFace, bottomRightFace});

    // 6 new half-edges (2 + 2 for our bars, 1 + 1 for our split edge)
    this->halfedges.insert({bottomLeft, upperHalf, upperLeft, topRight, lowerHalf, bottomRight});

    // 3 new edges (left bar, right bar, upper half of our "original" edge)
    newEdges.insert(newEdges.end(), {splitEdge, leftBar, rightBar});
    return newEdges;
}

void Mesh::loopSubdivide(int iterations) {
    // we will be mutating our edge vector, so need to make sure to not keep splitting!
    const int numEdges = this->edges.size();
    const int numFaces = this->faces.size();
    const int numVerts = this->vertices.size();
    const int numHedges = this->halfedges.size();

    for(int iter = 0; iter < iterations; iter++) {
        // sum up all vertex neighbors prior to edge split, as while neither split nor flip
        // should alter our degrees, it does alter our neighbor relationships
        for(const auto& vert : this->vertices) {
            int degree = 0;
            Vector3f sum = Vector3f(0, 0, 0);
            Halfedge* h = vert->halfedge;
            do {
                ++degree;
                sum += h->next->vertex->position;
                h = h->twin->next;
            } while(h != vert->halfedge);

            float degFactor = (degree == 3) ? (0.1875) : (
                (0.625 - pow(0.375 + 0.25 * cos(2 * M_PI / (float) degree), 2.0)) / (float) degree
            );

            // We cannot update our position at present, because our split verts still depend on it!
            vert->cached = (1 - degree * degFactor) * vert->position + degFactor * sum;
        }

        std::vector<Edge*> newEdges;
        newEdges.reserve(this->edges.size() * 3);

        for(const auto& originalEdge : this->edges) {
            std::vector<Edge*> constrEdges = this->edgeSplit(originalEdge, SplitMode::LOOP);
            newEdges.insert(newEdges.end(), constrEdges.begin(), constrEdges.end());
        }

        // After this point, all of our new vertices will be flagged as such

        this->edges.insert(newEdges.begin(), newEdges.end());

        for(int i = 0; i < newEdges.size(); i++) {
            // ignore our split edge
            if(i % 3 == 0) continue;

            Halfedge* root = newEdges[i]->halfedge;

            Vertex* v1 = root->vertex;
            Vertex* v2 = root->next->vertex;

            // only flip edges where where one is new, one is old (i.e. their flags are different)
            if(v1->flag != v2->flag) this->edgeFlip(newEdges[i]);
        }

        for(const auto& vert : this->vertices) {
            if(vert->flag) {
                Vertex* ln[2] = {nullptr, nullptr};
                Vertex* lr[2] = {nullptr, nullptr};

                Halfedge* h = vert->halfedge;

                int offset = 0;
                int noffset = 0;
                do {
                    if(h->next->vertex->flag) {
                        noffset++;
                        if(noffset % 2 == 1) ln[noffset / 2] = h->next->vertex;
                    } else {
                        lr[offset++] = h->next->vertex;
                    }

                    h = h->twin->next;
                } while(h != vert->halfedge);

                vert->position = 0.375 * (lr[0]->position + lr[1]->position);
                for(int i = 0; i < 2; i++) {
                    h = ln[i]->halfedge;
                    do {
                        Vertex* next = h->next->vertex;
                        if(!next->flag && next != lr[0] && next != lr[1]) {
                            vert->position += 0.125 * next->position;
                            break;
                        }

                        h = h->twin->next;
                    } while(h != ln[i]->halfedge);
                }
            }
        }

        for(const auto& vert : this->vertices) {
            if(vert->flag) {
                vert->flag = false;
            } else {
                vert->position = vert->cached;
            }
        }
    }

    // Modifying an iterable while being looped over can invalidate the iterator, so add our elements into our unordered_set now :)
    std::cout << "Loop subdivided " << iterations << " time(s)! Began with " <<
                 numEdges << " edges; now: " << this->edges.size() << ", " <<
                 numFaces << " faces; now: " << this->faces.size() << ", " <<
                 numVerts << " verts; now: " << this->vertices.size() << ", " <<
                 numHedges << " halfedges; now: " << this->halfedges.size() << std::endl;
}

auto argMin = [](float a, float b, float c) {
    if(a <= b && a <= c) return 0;
    else if(b <= a && b <= c) return 1;
    return 2;
};

void Mesh::updateFaceQuadric(
    Face* f,
    std::unordered_map<Face*, Matrix4f>& face_quadrics
) {
    Halfedge* root = f->halfedge;
    Vertex* v = root->vertex;

    Vector3f normal = (root->next->vertex->position - v->position).cross(
        root->next->next->vertex->position - v->position
    ).normalized();

    // (a, b, c, d) => (a, b, c, -(a, b, c).(x, y, z))
    Vector4f p;
    p << normal, -normal.dot(v->position);

    Matrix4f quad = p * p.transpose();
    face_quadrics[f] = quad;
}

void Mesh::updateVertexQuadric(
    Vertex* v,
    std::unordered_map<Vertex*, Matrix4f>& vertex_quadrics,
    std::unordered_map<Face*, Matrix4f>& face_quadrics
) {
    Halfedge* root = v->halfedge;
    Matrix4f error = Matrix4f::Zero();
    do {
        error += face_quadrics[root->face];
        root = root->twin->next;
    } while(root != v->halfedge);

    vertex_quadrics[v] = error;
}

void Mesh::updateEdgeError(
    Edge* edge,
    std::multimap<float, std::pair<Edge*, Vector3f>>& candidates,
    std::unordered_map<Vertex*, Matrix4f>& vertex_quadrics
) {
    // if this edge can't be collapsed now, it may be collapsible later; we add it to our queue
    // with weight infinity, and will likely re-weight it later as collapses happen
    if(!this->canCollapse(edge)) {
        candidates.insert({
            std::numeric_limits<float>::infinity(),
            {edge, edge->halfedge->vertex->position}
        });
        return;
    }

    Halfedge* root = edge->halfedge;

    Vertex* v1 = root->vertex;
    Vertex* v2 = root->twin->vertex;

    // Compute Q' by adding Q1 + Q2
    Matrix4f Qp = (vertex_quadrics[v1] + vertex_quadrics[v2]);

    Matrix4f QQ(Qp);
    QQ.row(3) = Vector4f(0, 0, 0, 1);

    // if non-invertible, go with min of: v1, v2, midpoint
    Matrix4f invQ;
    bool invertible;
    QQ.computeInverseWithCheck(invQ, invertible);
    if(!invertible) {
        Vector4f v1_h = v1->position.homogeneous();
        Vector4f v2_h = v2->position.homogeneous();
        Vector4f mid_h = (v1->position + v2->position / 2).homogeneous();

        float v1_error = (v1_h.transpose() * Qp * v1_h);
        float v2_error = (v2_h.transpose() * Qp * v2_h);
        float mid_error = (mid_h.transpose() * Qp * mid_h);

        int minArg = argMin(v1_error, v2_error, mid_error);
        switch(minArg) {
            case 0:
                candidates.insert({v1_error, {edge, v1->position}});
                break;
            case 1:
                candidates.insert({v2_error, {edge, v2->position}});
                break;
            case 2:
            default:
                candidates.insert({mid_error, {edge, mid_h.head<3>()}});
                break;
        }
    } else {
        Vector4f vp = invQ * Vector4f(0, 0, 0, 1);
        candidates.insert({vp.transpose() * Qp * vp, {edge, vp.head<3>()}});
    }
}


void Mesh::simplifyQuadric(int target) {
    std::cout << "Attempting to simplify mesh of " << this->faces.size() << " to " << target << " faces..." << std::endl;

    // no need to simplify if we're already at our target face count
    if(target > this->faces.size()) {
        std::cout << "No need to simplify, already at our target face count!" << std::endl;
        return;
    }

    // Compute the quadric of each triangular face, using a vertex to evaluate d
    std::unordered_map<Face*, Matrix4f> face_quadrics;
    for(const auto& face : this->faces) {
        updateFaceQuadric(face, face_quadrics);
    }

    // Compute the quadric of each vertex, using our computed face quadrics
    std::unordered_map<Vertex*, Matrix4f> quadrics;
    for(const auto& vert : this->vertices) {
        updateVertexQuadric(vert, quadrics, face_quadrics);
    }

    // map from minimum cost to Edge* and its corresponding v' to collapse to
    std::multimap<float, std::pair<Edge*, Vector3f>> candidates;
    for(const auto& edge : this->edges) {
        updateEdgeError(edge, candidates, quadrics);
    }

    // Okay, now we have our candidate edges; how do we collapse them?
    while(this->faces.size() > 4 && this->faces.size() > target && !candidates.empty()) {
        // This is the iterator to our lowest-cost, removable edge!
        auto it = candidates.begin();

        // If our weight is infinite, our lowest possible cost is infinity; hence, we can't collapse
        // any more of our edges, so we can end here
        if(it->first == std::numeric_limits<float>::infinity()) {
            break;
        }

        // Our edge is guaranteed to still exist (otherwise we would have removed it)
        std::pair<Edge*, Vector3f>& removable = it->second;

        Edge* r = removable.first;
        Vertex* B = r->halfedge->vertex;

        // Try and collapse our edge, and get the Edge* of our removals
        std::set<Edge*> removed = this->edgeCollapse(r, removable.second);

        // if edgeCollapse returns no edges, then the edge can no longer be removed; re-add it with infinite weight
        if(removed.size() == 0) {
            candidates.erase(it);
            // re-add our edge (which wasn't removable) with infinite weight, position irrelevant
            candidates.insert({std::numeric_limits<float>::infinity(), {r, B->position}});
            continue;
        }

        Halfedge* root = B->halfedge;

        // Recompute face quadrics of faces touching B, vert quadrics of all B's neighbors, and B's vertex quadric
        do {
            updateFaceQuadric(root->face, face_quadrics);
            root = root->twin->next;
        } while(root != B->halfedge);

        updateVertexQuadric(B, quadrics, face_quadrics);

        // Update the quadric of each of B's neighbors
        do {
            // Get the neighbor vertex
            updateVertexQuadric(root->next->vertex, quadrics, face_quadrics);
            root = root->twin->next;
        } while(root != B->halfedge);

        // After edge collapse, D should have been removed; we rebuild our "priority queue"
        std::multimap<float, std::pair<Edge*, Vector3f>> newCandidates;
        std::map<Edge*, float> removals;

        // "Erase" the element we just removed by progressing our iterator to start after it
        candidates.erase(it);

        for (auto newIt = candidates.begin(); newIt != candidates.end(); ++newIt) {
            Edge* e = newIt->second.first;

            // if our edge no longer exists we can just outright ignore it lol
            if(removed.contains(e)) {
                removals.insert({e, newIt->first});
                continue;
            }

            // Does this edge touch B? If so, we need to recompute its removal cost
            if(e->halfedge->vertex == B || e->halfedge->twin->vertex == B && !removals.contains(e)) {
                updateEdgeError(e, newCandidates, quadrics);
                removals.insert({e, newIt->first});
            }
        }

        for(const auto& r : removals) {
            // we're in a multimap so not guaranteed that this is unique!
            auto newIt = candidates.find(r.second);
            while(newIt->second.first != r.first) newIt++;
            candidates.erase(newIt);
        }

        for(const auto& n : newCandidates) {
            candidates.insert({n.first, n.second});
        }
    }

    std::cout << "Candidates: " << candidates.size() << std::endl;
    std::cout << "Simplified as much as possible! Faces: " << this->faces.size() << ", target: " << target << std::endl;
}

void Mesh::bilaterialDenoise(int iterations, float sigC, float sigS, float kernelWidth) {
    for(int i = 0; i < iterations; i++) {
        for(const auto& vert : this->vertices) {
            int faces = 0;
            Vector3f vertNorm = Vector3f::Zero();
            Halfedge* start = vert->halfedge;

            do {
                ++faces;
                vertNorm += (start->next->vertex->position - vert->position).cross(
                    start->next->next->vertex->position - vert->position
                ).normalized();

                start = start->twin->next;
            } while(start != vert->halfedge);

            vertNorm /= faces;

            std::unordered_set<Vertex*> visited = {vert};
            std::queue<Vertex*> process;
            process.push(vert);

            // BFS: Seed queue for 1-ring neighborhood
            do {
                Vertex* neighbor = start->next->vertex;
                float t = (vert->position - neighbor->position).norm();
                if(t < kernelWidth) {
                    process.push(neighbor);
                }
                start = start->twin->next;
            } while(start != vert->halfedge);

            float sum = 0;
            float normalizer = 0;

            // BFS: Loop over the queue of potential neighbors
            while(!process.empty()) {
                // Take our first neighbor
                Vertex* neighbor = process.front();

                // Throw it out if already processed
                if(visited.contains(neighbor)) {
                    process.pop();
                    continue;
                }

                // Otherwise: process (we only add to queue if it is within range, so no need to check here)
                float t = (vert->position - neighbor->position).norm();
                float h = vertNorm.dot(neighbor->position - vert->position);
                float wc = exp(-pow(t, 2) / (2 * pow(sigC, 2)));
                float ws = exp(-pow(h, 2) / (2 * pow(sigS, 2)));
                sum += (wc * ws) * h;
                normalizer += wc * ws;

                // Add all vertex neighbors to queue if within range
                Halfedge* c = neighbor->halfedge;
                do {
                    Vertex* n = c->next->vertex;
                    if((vert->position - n->position).norm() < kernelWidth) {
                        process.push(n);
                    }
                    c = c->twin->next;
                } while(c != neighbor->halfedge);

                visited.insert(neighbor);
                process.pop();
            }

            // Cache our vertex position, since calculations need to use original mesh position
            vert->cached = vert->position + vertNorm * (sum / (normalizer != 0 ? normalizer : 1));
        }

        for(const auto& vert : this->vertices) {
            vert->position = vert->cached;
        }
    }
}

void Mesh::noisify(float weight) {
    for(const auto& vert : this->vertices) {
        Halfedge* root = vert->halfedge;
        vert->cached = (weight * (RNG() - 0.5)) * (
            root->next->vertex->position - vert->position
        ).cross(
            root->next->next->vertex->position - vert->position
        ).normalized();
    }

    for(const auto& vert : this->vertices) {
        vert->position += vert->cached;
    }
}

Mesh::Mesh(Mesh& m, std::map<Halfedge*, Halfedge*>& mapHedges) {
    // We need to create a deepcopy of all our structs, so need to map ptr -> ptr
    this->halfedges.reserve(m.halfedges.size());
    this->edges.reserve(m.edges.size());
    this->faces.reserve(m.faces.size());
    this->vertices.reserve(m.vertices.size());

    for(const auto& old : m.halfedges) {
        Halfedge* nu = new Halfedge();
        mapHedges[old] = nu;
        this->halfedges.insert(nu);
    }

    for(const auto& old : m.edges) {
        // Get the remapped edge for our edge (and its twin)
        Halfedge* remapped = mapHedges[old->halfedge];
        Halfedge* twinRemapped = mapHedges[old->halfedge->twin];

        Edge* nu = new Edge(remapped);

        remapped->twin = twinRemapped;
        twinRemapped->twin = remapped;

        remapped->edge = nu;
        twinRemapped->edge = nu;

        this->edges.insert(nu);
    }

    for(const auto& old : m.faces) {
        Halfedge* remapped = mapHedges[old->halfedge];
        Halfedge* remappedNext = mapHedges[old->halfedge->next];
        Halfedge* remappedLast = mapHedges[old->halfedge->next->next];

        Face* nu = new Face(remapped);

        remapped->face = nu;
        remappedNext->face = nu;
        remappedLast->face = nu;

        remapped->next = remappedNext;
        remappedNext->next = remappedLast;
        remappedLast->next = remapped;

        this->faces.insert(nu);
    }

    for(const auto& old : m.vertices) {
        Halfedge* remapped = mapHedges[old->halfedge];

        Vertex* nu = new Vertex(old->position, remapped);

        do {
            remapped->vertex = nu;
            remapped = remapped->twin->next;
        } while(remapped != nu->halfedge);

        this->vertices.insert(nu);
    }
}

void Mesh::createHalfedge(
    const std::vector<Vector3f>& vs,
    const std::vector<Vector3i>& fs
) {
    if(vs.size() == 0) throw std::invalid_argument("Vibes");

    // initialize our verts to null, since we need to read these back when checking for existence
    std::vector<Vertex*> newVertices = std::vector<Vertex*>(vs.size(), nullptr);
    this->vertices.reserve(vs.size());

    // reserve our faces, so we can use pushback (we don't know our index)
    this->faces.reserve(fs.size());

    int eulerCharacteristic = fs.size() + vs.size() - 2;

    // use temporary maps to track which halfedges we've already built (since we need to twin later)
    std::map<std::tuple<int, int>, Edge*> edgeMap;
    std::map<std::tuple<int, int>, Halfedge*> halfedgeMap;

    this->edges.reserve(eulerCharacteristic);

    // We will have 2x halfedges vs edges
    this->halfedges.reserve(2 * eulerCharacteristic);

    // iterate over our faces, creating halfedges for each: [(A, B), (B, C), (C, A)]
    for(const auto& face : fs) {
        // Halfedges: h_ab, h_bc, h_ca
        Halfedge* face_hes[3] = {new Halfedge(), new Halfedge, new Halfedge()};
        Face* f = new Face(face_hes[0]);
        faces.insert(f);

        for(int i = 0; i < 3; i++) {
            Halfedge* curr = face_hes[i];
            Halfedge* next = face_hes[(i + 1) % 3];

            int curr_vert = face[i];
            int next_vert = face[(i + 1) % 3];

            Vertex* nv;
            if(!newVertices[curr_vert]) {
                nv = new Vertex(vs[curr_vert], curr);
                newVertices[curr_vert] = nv;
            } else {
                // if this vertex already exists, we don't need to recreate it, though it points to a different HE
                nv = newVertices[curr_vert];
            }

            Edge* ne;
            // sort for our edge key (since we need a consistent order)
            std::tuple<int, int> k_edge = curr_vert < next_vert ? std::tuple<int, int>{curr_vert, next_vert} :
                                                                  std::tuple<int, int>{next_vert, curr_vert};
            auto find = edgeMap.find(k_edge);
            if(find == edgeMap.end()) {
                ne = new Edge(curr);
                edgeMap[k_edge] = ne;
                this->edges.insert(ne);
            } else {
                ne = find->second;
            }

            curr->vertex = nv;
            curr->edge = ne;
            curr->face = f;
            curr->next = next;

            if(halfedgeMap.contains({curr_vert, next_vert})) {
                std::cout << "MAYDAY MAYDAY" << std::endl;
                throw std::invalid_argument("how tf we got this alr");
            }

            halfedgeMap[{curr_vert, next_vert}] = curr;
            this->halfedges.insert(curr);
        }
    }

    this->vertices.insert(newVertices.begin(), newVertices.end());

    for(const auto& [k, v] : halfedgeMap) {
        auto [src, dest] = k;

        // do we already have a twin? if so, save time, don't need to set our twin's twin to us
        if(v->twin) {
            if(v->twin->twin == v) continue;
            throw std::invalid_argument("Twin twin should be us?");
        }

        auto found_twin = halfedgeMap.find({dest, src});
        if(found_twin == halfedgeMap.end()) {
            std::cout << "No twin exists for HE (" << src << ", " << dest << ")" << std::endl;
            throw std::invalid_argument("wtf happened babes");
        } else {
            v->twin = found_twin->second;
            v->twin->twin = v;
        }
    }

    std::cout << "Halfedge construction completed: " <<
                 this->faces.size() << " faces [E: " << fs.size() << "], " <<
                 this->vertices.size() << " verts [E: " << vs.size() << "], " <<
                 this->edges.size() << " edges [E: " << eulerCharacteristic << "], and " <<
                 this->halfedges.size() << " halfedges [E: " << 2 * eulerCharacteristic << "]" << std::endl;
}

void Mesh::createHalfedge() { createHalfedge(this->_vertices, this->_faces); }

Mesh::~Mesh() {
    // All geometry nodes are dynamically allocated, and need to be freed
    for(auto* v : this->vertices) delete v;
    for(auto* f : this->faces) delete f;
    for(auto* e : this->edges) delete e;
    for(auto* h : this->halfedges) delete h;
}

// OBJ load stuff
void Mesh::loadFromFile(const string &filePath)
{
    tinyobj::attrib_t attrib;
    vector<tinyobj::shape_t> shapes;
    vector<tinyobj::material_t> materials;

    QFileInfo info(QString(filePath.c_str()));
    string err;
    bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &err,
                                info.absoluteFilePath().toStdString().c_str(), (info.absolutePath().toStdString() + "/").c_str(), true);
    if (!err.empty()) {
        cerr << err << endl;
    }

    if (!ret) {
        cerr << "Failed to load/parse .obj file" << endl;
        return;
    }

    for(size_t s = 0; s < shapes.size(); s++) {
        size_t index_offset = 0;
        for(size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
            unsigned int fv = shapes[s].mesh.num_face_vertices[f];

            Vector3i face;
            for(size_t v = 0; v < fv; v++) {
                tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];

                face[v] = idx.vertex_index;

            }
            _faces.push_back(face);

            index_offset += fv;
        }
    }
    for (size_t i = 0; i < attrib.vertices.size(); i += 3) {
        _vertices.emplace_back(attrib.vertices[i], attrib.vertices[i + 1], attrib.vertices[i + 2]);
    }
    cout << "Loaded " << _faces.size() << " faces and " << _vertices.size() << " vertices" << endl;
}

void Mesh::saveToFile(const string &filePath)
{
    ofstream outfile;
    outfile.open(filePath);

    // Halfedge vertices are not numbered, so we need to map vertex indices correctly
    std::map<const Vertex*, int> inds;

    // Obj vertices start @ 1
    int vertIdx = 1;
    for(const auto& vert : this->vertices) {
        const Vector3f& v = vert->position;
        inds[vert] = vertIdx++;
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write faces
    for(const auto& face : this->faces) {
        outfile << "f " << (inds[face->halfedge->vertex]) << " "
                        << (inds[face->halfedge->next->vertex]) << " "
                        << (inds[face->halfedge->next->next->vertex]) << endl;
    }

    outfile.close();
}
