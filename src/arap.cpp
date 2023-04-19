#include "arap.h"
#include "graphics/meshloader.h"
#include "QtCore/qcommandlineparser.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>

using namespace std;
using namespace Eigen;

ARAP::ARAP() {}

void ARAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax)
{
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;

    QCommandLineParser parser;
    parser.setApplicationDescription("Stamortack: Interactive Cubosity Generator");
    parser.addHelpOption();
    parser.process(*QCoreApplication::instance());
    if(parser.positionalArguments().length() < 1) {
        std::cerr << "Stamortack requires a valid trimesh in .obj format!" << std::endl;
        exit(1);
    }

    // If this doesn't work for you, remember to change your working directory
    if (MeshLoader::loadTriMesh(parser.positionalArguments().at(0).toStdString(), vertices, triangles)) {
        // our adjacency structure maps each vertex to its adjacent vertices
        this->adj = vector<std::map<Vindex, std::pair<Vindex, Vindex>>>(vertices.size());
        this->remap = vector<int>(vertices.size());
        this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
        this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
        this->cached_positions = vertices;
        this->mesh.initFromVectors(vertices, triangles);

        this->m_shape.init(vertices, triangles);
        this->computeAdjacency();
    }

    // Students, please don't touch this code: get min and max for viewport stuff
    MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
    int i = 0;
    for (unsigned long i = 0; i < vertices.size(); ++i) {
        all_vertices.row(i) = vertices[i];
    }

    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();
}

void ARAP::computeAdjacency() {
    const vector<Vector3f>& vertices = m_shape.getVertices();
    const vector<Vector3i>& faces = m_shape.getFaces();

    this->adj.clear();
    this->adj.resize(vertices.size());
    vector<bool> initialized(vertices.size(), false);

    for(int i = 0; i < faces.size(); i++) {
        const auto& tri = faces[i];
        for(int v = 0; v < 3; v++) {
            int vert = tri[v];
            int nextVert = tri[(v + 1) % 3];
            int lastVert = tri[(v + 2) % 3];
            if(initialized[vert]) {
                // neighbor vertices and their wing vertices, as we need to compute cotangent angles
                auto& av = this->adj[vert];
                if(av.contains(nextVert)) {
                    av[nextVert].second = lastVert;
                } else {
                    av[nextVert].first = lastVert;
                }

                if(av.contains(lastVert)) {
                    av[lastVert].second = nextVert;
                } else {
                    av[lastVert].first = nextVert;
                }

            } else {
                // first tri encountered with this vert, so adjacencies are:
                // {nextVert, lastVert} for verts, {i} for tris
                this->adj[vert] = {
                    // every edge is shared by exactly 2 faces, so we need to find the other wing
                    // vert; we are guaranteed to see this later
                    {{nextVert, {lastVert, -1}},
                     {lastVert, {nextVert, -1}}}
                };

                initialized[vert] = true;
            }
        }
    }
}

void ARAP::precompute() {
    const std::unordered_set<int>& anchors = this->m_shape.getAnchors();
    if(this->modified || this->cached_anchors != anchors) {
        this->cached_anchors = this->m_shape.getAnchors();
        this->cached_positions = this->m_shape.getVertices();

        for(int i = 0, r = 0; i < this->adj.size(); i++) {
            if(!anchors.contains(i)) {
                this->remap[i] = r++;
            } else {
                this->remap[i] = -1;
            }
        }

        this->computeWeights(cached_positions);
        this->computeSystem();
    }

    this->modified = false;
}

void ARAP::computeWeights(const auto& verts) {
    this->W.setZero();
    for(int v = 0; v < verts.size(); v++) {
        const Vector3f& vert = verts[v];
        for(const auto& [n, wings] : this->adj[v]) {
            const Vector3f& neighbor = verts[n];
            const Vector3f& left = verts[wings.first];
            const Vector3f& right = verts[wings.second];

            Vector3f a1 = vert - left;
            Vector3f a2 = neighbor - left;
            Vector3f b1 = vert - right;
            Vector3f b2 = neighbor - right;

            this->W.coeffRef(v, n) = 0.5 * (
                abs((a1.dot(a2) / (a1.cross(a2)).norm())) +
                abs((b1.dot(b2) / (b1.cross(b2)).norm()))
            );
        }
    }
}

void ARAP::computeRotations(const auto& newVerts, const int mover, const Vector3f& moved) {
    for(int v = 0; v < this->adj.size(); v++) {
        // Our covariance matrix is the sum of our neighbor weights times the edge relationships
        int r = this->remap[v];

        Matrix3f cov = Matrix3f::Zero();
        const Vector3f& vert = this->cached_positions[v];

        // if our current cell is an anchor, it should remain fixed, meaning e and e' should be the same;
        // we reference its position in originalVerts, as newVerts won't have an entry for it
        const Vector3f& newVert = (r != -1) ? newVerts.row(r) : (
            (v == mover) ? moved : this->cached_positions[v]
        );

        for(const auto& [neighbor, _] : this->adj[v]) {
            const Vector3f& neigh = this->cached_positions[neighbor];

            int n = this->remap[neighbor];
            const Vector3f& newNeigh = (n != -1) ? newVerts.row(n) : (
                (neighbor == mover) ? moved : this->cached_positions[neighbor]
            );

            cov += this->W.coeff(v, neighbor) * (vert - neigh) * (newVert - newNeigh).transpose();
        }

        auto svd = cov.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
        Matrix3f rot = svd.matrixV() * svd.matrixU().transpose();

        // if our determinant is negative, we found our reflection matrix instead of our rotation one;
        // invert our smallest singular value, per the paper
        if(rot.determinant() < 0) {
            auto newU = svd.matrixU();
            // Singular values are always sorted in decreasing order, so, idk, frick the haters
            newU.col(2) *= -1;
            rot = svd.matrixV() * newU.transpose();
        }

        this->rotations[v] = rot;
    }
}

void ARAP::computeSystem() {
    const std::unordered_set<int>& anchors = this->m_shape.getAnchors();
    this->L = SparseMatrix<float>(this->adj.size() - anchors.size(), this->adj.size() - anchors.size());
    this->L.setZero();
    for(int v = 0; v < this->adj.size(); v++) {
        int r = this->remap[v];
        if(r == -1) continue;

        for(const auto& [neighbor, _] : this->adj[v]) {
            int n = this->remap[neighbor];

            float weight = this->W.coeff(v, neighbor);
            this->L.coeffRef(r, r) += weight;
            if(n != -1) {
                this->L.coeffRef(r, n) -= weight;
            }
        }
    }

    this->sal.compute(this->L);
    if (this->sal.info() != Eigen::Success) {
        std::cout << "Error: Solver could not compute" << std::endl;
    }
}

// Move an anchored vertex, defined by its index, to targetPosition
void ARAP::move(int vertex, Vector3f targetPosition) {
    std::vector<Eigen::Vector3f> new_vertices = m_shape.getVertices();
    const std::unordered_set<int>& anchors = m_shape.getAnchors();

    this->modified = true;
    new_vertices[vertex] = targetPosition;

    MatrixXf estimate = MatrixXf::Zero(this->adj.size() - anchors.size(), 3);
    for(int v = 0; v < this->adj.size(); v++) {
        int r = this->remap[v];
        if(r != -1) {
            estimate.row(r) = new_vertices[v];
        }
    }

    for(int iterations = 0; iterations < this->NUM_ITERATIONS; iterations++) {
        this->computeRotations(estimate, vertex, targetPosition);

        MatrixXf b = MatrixXf::Zero(this->adj.size() - anchors.size(), 3);
        for(int i = 0; i < this->adj.size(); i++) {
            int r = this->remap[i];

            // this is an anchor; no entry in b
            if(r == -1) continue;
            Vector3f entry = Vector3f::Zero();
            for(const auto& [neigh, _] : this->adj[i]) {
                float w = this->W.coeff(i, neigh);

                int n = this->remap[neigh];
                if(n == -1) entry += w * new_vertices[neigh];
                entry += 0.5 * w
                             * (this->rotations[i] + this->rotations[neigh])
                             * (cached_positions[i] - cached_positions[neigh]);
            }

            b.row(r) = entry;
        }

        estimate = this->sal.solve(b);
    }


    for(int i = 0; i < this->adj.size(); i++) {
        int r = this->remap[i];

        // no need to move
        if(r == -1) continue;
        new_vertices[i] = estimate.row(r);
    }

    m_shape.setVertices(new_vertices);
}

void ARAP::subdivide() {
    mesh.subdivide();

    const vector<Vector3f>& vertices = mesh.getVertices();
    const vector<Vector3i>& faces = mesh.getFaces();

    m_shape.init(vertices, faces);
    computeAdjacency();
    this->remap = vector<int>(vertices.size());
    this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
    this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
    this->cached_positions = vertices;
}

void ARAP::denoise(Settings& s) {
    mesh.denoise(s.denoiseDistance, s.denoiseSigma1, s.denoiseSigma2);

    const vector<Vector3f>& vertices = mesh.getVertices();
    const vector<Vector3i>& faces = mesh.getFaces();

    m_shape.init(vertices, faces);
    computeAdjacency();
    this->remap = vector<int>(vertices.size());
    this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
    this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
    this->cached_positions = vertices;
}



void ARAP::simplify(Settings& s) {
    mesh.simplify(s.simplifyTarget);

    const vector<Vector3f>& vertices = mesh.getVertices();
    const vector<Vector3i>& faces = mesh.getFaces();

    m_shape.init(vertices, faces);
    computeAdjacency();
    this->remap = vector<int>(vertices.size());
    this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
    this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
    this->cached_positions = vertices;
}
