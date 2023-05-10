#include "arap.h"
#include "graphics/meshloader.h"
#include "QtCore/qcommandlineparser.h"
#include "OsqpEigen/OsqpEigen.h"

#include <iostream>
#include <set>
#include <map>
#include <vector>
#include <filesystem>
#include <fstream>

using namespace std;
using namespace Eigen;

ARAP::ARAP() {}

void ARAP::init(Eigen::Vector3f &coeffMin, Eigen::Vector3f &coeffMax, Settings& settings)
{
    vector<Vector3f> vertices;
    vector<Vector3i> triangles;
    vector<Vector2f> uv;
    vector<Vector3f> vertexColors;

    string texture;

    QCommandLineParser parser;
    parser.setApplicationDescription("Stamortack: Interactive Cubosity Generator");
    parser.addHelpOption();
    parser.process(*QCoreApplication::instance());
    if(parser.positionalArguments().length() < 1) {
        std::cerr << "Stamortack requires a valid trimesh in .obj format!" << std::endl;
        exit(1);
    }

    string obj = parser.positionalArguments().at(0).toStdString();
    settings.meshPath = obj;

    if(filesystem::is_directory(obj)) {
        auto meshfile = filesystem::path(obj) / "mesh.obj";
        auto stampfile = filesystem::path(obj) / "mesh.stamp";
        if(!(filesystem::exists(meshfile) && filesystem::exists(stampfile))) {
            std::cerr << "Provided directories must contain both simplified mesh (.obj) and associated reconstruction (.stamp) files!" << std::endl;
            exit(1);
        }

        if(MeshLoader::loadTriMesh(meshfile, vertices, triangles, uv, texture)) {
            // our adjacency structure maps each vertex to its adjacent vertices
            bool loaded = this->mesh.loadProgressiveMesh(stampfile, vertices, triangles);
            if(!loaded) {
                std::cerr << "Failed to load progressive mesh :(" << std::endl;
                exit(1);
            }

            // I don't trust that these vertices aren't mangled somehow...
            vertices = this->mesh.getVertices();
            triangles = this->mesh.getFaces();

            for(int i = 0; i < vertices.size(); i++) {
                auto& color = settings.orientationGroups[this->mesh.getOrientationGroup(i)]->color;
                vertexColors.push_back({color.redF(), color.greenF(), color.blueF()});
            }

            this->adj = vector<std::map<Vindex, std::pair<Vindex, Vindex>>>(vertices.size());
            this->remap = vector<int>(vertices.size());
            this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
            this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
            this->cubeData = vector<CubeData>(vertices.size(), CubeData{});
            this->cached_positions = vertices;

            if (texture.size() != 0) {
                QString objPath = QString(obj.data());
                QString path = objPath.left(objPath.lastIndexOf('/'));
                path += "/";
                path.append(texture.data());

                this->m_shape.init(vertices, triangles, uv, vertexColors, path.toStdString());
            } else {
                this->m_shape.init(vertices, triangles, vertexColors);
            }

            this->computeAdjacency();
        }

    } else {
        if (MeshLoader::loadTriMesh(obj, vertices, triangles, uv, texture)) {
            // our adjacency structure maps each vertex to its adjacent vertices
            this->adj = vector<std::map<Vindex, std::pair<Vindex, Vindex>>>(vertices.size());
            this->remap = vector<int>(vertices.size());
            this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
            this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
            this->cubeData = vector<CubeData>(vertices.size(), CubeData{});
            this->cached_positions = vertices;
            this->mesh.initFromVectors(vertices, triangles);

            for(int i = 0; i < vertices.size(); i++) {
                auto& color = settings.orientationGroups[this->mesh.getOrientationGroup(i)]->color;
                vertexColors.push_back({color.redF(), color.greenF(), color.blueF()});
            }

            if (texture.size() != 0) {
                QString objPath = QString(obj.data());
                QString path = objPath.left(objPath.lastIndexOf('/'));
                path += "/";
                path.append(texture.data());

                this->m_shape.init(vertices, triangles, uv, vertexColors, path.toStdString());
            } else {
                this->m_shape.init(vertices, triangles, vertexColors);
            }

            this->computeAdjacency();
        }
    }

//    std::cout << "this gonna take a minuteeeee..." << std::endl;
//    this->mesh.simplify(1000);

    // Students, please don't touch this code: get min and max for viewport stuff
    MatrixX3f all_vertices = MatrixX3f(vertices.size(), 3);
    int i = 0;
    for (unsigned long i = 0; i < vertices.size(); ++i) {
        all_vertices.row(i) = vertices[i];
    }

    coeffMin = all_vertices.colwise().minCoeff();
    coeffMax = all_vertices.colwise().maxCoeff();

    this->modified = true;
    this->precompute();
}

void ARAP::computeAdjacency() {
    const vector<Vector3f>& vertices = m_shape.getVertices();
    const vector<Vector3i>& faces = m_shape.getFaces();

    this->adj.clear();
    this->adj.resize(vertices.size());

    this->faceAdj.clear();
    this->faceAdj.resize(vertices.size());

    vector<bool> initialized(vertices.size(), false);

    for(int i = 0; i < faces.size(); i++) {
        const auto& tri = faces[i];
        for(int v = 0; v < 3; v++) {
            int vert = tri[v];
            int nextVert = tri[(v + 1) % 3];
            int lastVert = tri[(v + 2) % 3];

            // for each vertex, this face is adjacent to it
            this->faceAdj[vert].push_back(i);

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
        this->getPerVertexInfo();
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

void ARAP::computeCubeRotations(const auto& newVerts, Settings& settings) {
    for (int v = 0; v < this->adj.size(); v++) {
        const Vector3f& vertex = this->cached_positions[v];

        int remappedVertex = this->remap[v];
        const Vector3f& newVertex = (remappedVertex != -1) ? newVerts.row(remappedVertex) : this->cached_positions[v];

        auto& neighborhood = this->adj[v];
        CubeData& currentCubeData = this->cubeData[v];
        Vector3f& vertexNormal = this->normals[v];
        float vertexArea = this->areas[v];

//        std::cout << "Vertex: " << v << std::endl;
//        std::cout << "V: " << vertex << std::endl;
//        std::cout << "Normal: " << vertexNormal << std::endl;
//        std::cout << "Area: " << vertexArea << "\n" << std::endl;

        // Compute the D and D' matrices
        // They are 3xN(i) where D is the old positions and D' is the new vertex positions of the neighbors
        MatrixXf D = MatrixXf::Zero(3, neighborhood.size());
        MatrixXf Dprime = MatrixXf::Zero(3, neighborhood.size());

        // The neighborhoodW matrix is a subsection of the W matrix that just has weights for neighbors on the diagonal
        MatrixXf neighborhoodW = MatrixXf::Zero(neighborhood.size(), neighborhood.size());

        int neighborNum = 0;
        for(const auto& [neighborIndex, _] : this->adj[v]) {
            const Vector3f& oldNeighborPos = this->cached_positions[neighborIndex];
            D.col(neighborNum) = oldNeighborPos - vertex;

            int remappedNeighbor = this->remap[neighborIndex];
            const Vector3f& newNeighborPos = (remappedNeighbor != -1) ? newVerts.row(remappedNeighbor) : this->cached_positions[neighborIndex]; // don't allow anchors to move
            Dprime.col(neighborNum) = newNeighborPos - newVertex;

            neighborhoodW(neighborNum, neighborNum) = W.coeff(v, neighborIndex);

            neighborNum++;
        }

        // FROM REFERENCE IMPLEMENTATION:
        // Note:
        // M = [D n] * [W 0; 0 rho] * [Dprime (z-u)]'
        //   = D * W * Dprime' + n * rho * (z-u)'
        //   = Mpre + n * rho * (z-u)'
        MatrixXf Mprecomputed = D * neighborhoodW * Dprime.transpose();

//        std::cout << "D:" << D << std::endl;
//        std::cout << "D': " << Dprime << std::endl;
//        std::cout << "Precomputed: " << Mprecomputed << std::endl;

        Matrix3f R;
        for (int i = 0; i < 100; i++) {
            MatrixXf M = Mprecomputed + (currentCubeData.rho * vertexNormal * (currentCubeData.z - currentCubeData.u).transpose());

            auto svd = M.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
            R = svd.matrixV() * svd.matrixU().transpose();

            // if our determinant is negative, we found our reflection matrix instead of our rotation one;
            // invert our smallest singular value, per the paper
            if(R.determinant() < 0) {
                auto newU = svd.matrixU();
                // Singular values are always sorted in decreasing order
                newU.col(2) *= -1;
                R = svd.matrixV() * newU.transpose();
            }

            if (R.determinant() < 0) {
                std::cout << "AHH" << std::endl;
            }

            float LAMBDA = settings.orientationGroups[this->mesh.getOrientationGroup(v)]->lambda->value();
            float MU = 10.f; // NOTE: pass in as setting eventually;
            float TAU = 2.f; // NOTE: pass in as a setting eventually;
            float EPSILON_ABSOLUTE = 1e-6f; // value specified in paper
            float EPSILON_RELATIVE = 1e-3f; // value specified in paper

            MatrixXd B = settings.orientationGroups[this->mesh.getOrientationGroup(v)]->rotation;
            int m = B.rows();

            // Z step
            // Shrinkage step can be solved with Sκ(a)=(a − κ)+ − (−a − κ)+.
            // credit: https://web.stanford.edu/~boyd/papers/pdf/admm_distr_stats.pdf
            // Section 4.3
            // k = lambda * area / rho
            // a = Ri * normal + u
            Vector3f prevZ = currentCubeData.z;
//            Vector3f a = R * vertexNormal + currentCubeData.u;
//            float k = LAMBDA * vertexArea / currentCubeData.rho;
//            currentCubeData.z = (a.array() - k).max(0.f) - (-a.array() - k).max(0.f);

            // Z step with shape generalization
            OsqpEigen::Solver solver;

            SparseMatrix<double> P(3 + m, 3 + m);
            P.reserve(VectorXi::Constant(4, 2));
            P.insert(0, 0) = (double) currentCubeData.rho;
            P.insert(1, 1) = (double) currentCubeData.rho;
            P.insert(2, 2) = (double) currentCubeData.rho;

            VectorXd q (3 + m);
            VectorXd leftSide = -((double) currentCubeData.rho) * (R * vertexNormal + currentCubeData.u).cast<double>();
            VectorXd rightSide = (double) (LAMBDA * vertexArea) * VectorXd::Ones(m);
            q << leftSide, rightSide;

            MatrixXd ADense(2*m, 3 + m);
            ADense.block(0, 0, m, 3) = B;
            ADense.block(m, 0, m, 3) = -B;
            ADense.block(0, 3, m, m) = -MatrixXd::Identity(m, m);
            ADense.block(m, 3, m, m) = -MatrixXd::Identity(m, m);

            SparseMatrix<double> A(2*m, 3 + m);
            vector<Triplet<double>> coefficients;
            for (int i = 0; i < A.rows(); i++) {
                for (int j = 0; j < A.cols(); j++) {
                    if (ADense(i, j) == 0.f) {
                        continue;
                    }

                    coefficients.push_back({ i, j, ADense(i, j)});
                }
            }
            A.setFromTriplets(coefficients.begin(), coefficients.end());

            VectorXd upperBound = VectorXd::Zero(2*m, 1);
            VectorXd lowerBound = -(std::numeric_limits<double>::max() * VectorXd::Ones(2*m, 1));

            solver.settings()->setVerbosity(false);
            solver.data()->setNumberOfVariables(3 + m);
            solver.data()->setHessianMatrix(P);
            solver.data()->setGradient(q);

            solver.data()->setNumberOfConstraints(2 * m);
            solver.data()->setLinearConstraintsMatrix(A);
            solver.data()->setLowerBound(lowerBound);
            solver.data()->setUpperBound(upperBound);

            if(!solver.initSolver()) {
                std::cout << "AHH" << std::endl;
            };

            if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) {
                std::cout << "AHH 2" << std::endl;
            };

            Eigen::VectorXd QPSolution;
            QPSolution = solver.getSolution();
            currentCubeData.z = QPSolution.block(0, 0, 3, 1).cast<float>();

            // U step
            // u^k+1 = u^k + Ri * normal - z
            currentCubeData.u += R * vertexNormal - currentCubeData.z;


            // RHO and U step
            // credit: https://web.stanford.edu/~boyd/papers/pdf/admm_distr_stats.pdf
            // Section 3.4.1
            float rResidual = (currentCubeData.z - R* vertexNormal).norm();
            float sResidual = (-currentCubeData.rho * (currentCubeData.z - prevZ)).norm();

            if (rResidual > MU * sResidual) {
                currentCubeData.rho *= TAU;
                currentCubeData.u /= TAU;
            } else if (sResidual > MU * rResidual) {
                currentCubeData.rho /= TAU;
                currentCubeData.u *= TAU;
            }

            // Stopping Criteria
            // credit: https://web.stanford.edu/~boyd/papers/pdf/admm_distr_stats.pdf
            // Section 3.3.1

            // epsilonPrimal = sqrt(p)*epsilonAbsolute + epsilonRelative*max{||x||, ||z||, ||c=0||}
            // epsilonDual = sqrt(n)*epsolonAbsolute + epsilonRelative*(||rho*u||)
            // where n = 3 (dimensionality of R)
            // where p = 6 (no clue where this comes from aparrently A is pxn but so I would think its 3 but their reference code multiplies by 2

            float n = (float) R.cols();
            float p = 2.f * n;
            float epsilonPrimal = sqrtf(p)*EPSILON_ABSOLUTE + EPSILON_RELATIVE * max((R * vertexNormal).norm(), currentCubeData.z.norm());
            float epsilonDual = sqrtf(n)*EPSILON_ABSOLUTE + EPSILON_RELATIVE * (currentCubeData.rho * currentCubeData.u).norm();

            if (rResidual < epsilonPrimal && sResidual < epsilonDual) {
                break;
            }
        }

        this->rotations[v] = R;
    }
}

void ARAP::getPerVertexInfo() {
    const std::vector<Vector3i>& faces = m_shape.getFaces();
    this->normals.clear();
    this->normals.resize(this->cached_positions.size());

    this->areas.clear();
    this->areas.resize(this->cached_positions.size());

    for(int i = 0; i < faces.size(); i++) {
        std::cout << getFaceNormal(faces[i]) << std::endl;
    }

    for (int vert = 0; vert < this->cached_positions.size(); vert++) {
         std::vector<int>& adjacentFaces = this->faceAdj[vert];
         Vector3f vertexNormal = Vector3f::Zero();
         float vertexArea = 0.f;

         for (int faceIndex : adjacentFaces) {
            const Vector3i& face = faces[faceIndex];

            // NORMAL
            Vector3f faceNormal = getFaceNormal(face);
            vertexNormal += faceNormal;

            // AREA
            vertexArea += getFaceArea(face);
         }

         vertexNormal /= adjacentFaces.size();
         this->normals[vert] = vertexNormal;

         vertexArea /= adjacentFaces.size();
         this->areas[vert] = vertexArea ;
    }
}

Vector3f ARAP::getFaceNormal(const Vector3i& face) {
    Vector3f& v1 = this->cached_positions[face[0]];
    Vector3f& v2 = this->cached_positions[face[1]];
    Vector3f& v3 = this->cached_positions[face[2]];

    Vector3f e1 = v2 - v1;
    Vector3f e2 = v3 - v1;
    Vector3f n = e1.cross(e2);
    return n.normalized();
}

float ARAP::getFaceArea(const Vector3i& face) {
    // Credit: https://math.stackexchange.com/questions/128991/how-to-calculate-the-area-of-a-3d-triangle
    Vector3f& v1 = this->cached_positions[face[0]];
    Vector3f& v2 = this->cached_positions[face[1]];
    Vector3f& v3 = this->cached_positions[face[2]];

    Eigen::Vector3f sideA = v2 - v1;
    Eigen::Vector3f sideB = v3 - v1;

    return sideA.cross(sideB).norm();
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

    std::cout << "System has " << this->L.rows() << " rows" << std::endl;

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

void ARAP::cubify(int iters, Settings& settings) {
    std::vector<Eigen::Vector3f> new_vertices = m_shape.getVertices();
    const std::unordered_set<int>& anchors = m_shape.getAnchors();

    MatrixXf estimate = MatrixXf::Zero(this->adj.size() - anchors.size(), 3);
    for(int v = 0; v < this->adj.size(); v++) {
        int r = this->remap[v];
        if(r != -1) {
            estimate.row(r) = new_vertices[v];
        }
    }

//    std::cout << "Estimate has " << estimate.rows() << " rows" << std::endl;

    for (int iterations = 0; iterations < iters; iterations++) {
        computeCubeRotations(estimate, settings);

//        std::cout << "Rotation:\n" << rotations[0] << std::endl;
//        break;

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

        if (iterations % 50 == 0) {
            std::cout << iterations << std::endl;
        }
    }

    for(int i = 0; i < this->adj.size(); i++) {
        int r = this->remap[i];

        // no need to move
        if(r == -1) continue;
        new_vertices[i] = estimate.row(r);
    }

    mesh.updatePositions(new_vertices);
    m_shape.setVertices(new_vertices);

//    computeAdjacency();
//    this->remap = vector<int>(vertices.size());
//    this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
//    this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
//    this->cached_positions = vertices;
}

void ARAP::subdivide(Settings& settings) {
    mesh.subdivide();

    const vector<Vector3f>& vertices = mesh.getVertices();
    const vector<Vector3i>& faces = mesh.getFaces();

    vector<Vector3f> vertexColors; vertexColors.reserve(vertices.size());

    for(int i = 0; i < vertices.size(); i++) {
        auto& color = settings.orientationGroups[this->mesh.getOrientationGroup(i)]->color;
        vertexColors.push_back({color.redF(), color.greenF(), color.blueF()});
    }

    m_shape.init(vertices, faces, vertexColors);
    computeAdjacency();
    this->remap = vector<int>(vertices.size());
    this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
    this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
    this->cached_positions = vertices;
}

void ARAP::denoise(Settings& settings) {
    mesh.denoise(settings.denoiseDistance, settings.denoiseSigma1, settings.denoiseSigma2);

    const vector<Vector3f>& vertices = mesh.getVertices();
    const vector<Vector3i>& faces = mesh.getFaces();

    vector<Vector3f> vertexColors; vertexColors.reserve(vertices.size());
    for(int i = 0; i < vertices.size(); i++) {
        auto& color = settings.orientationGroups[this->mesh.getOrientationGroup(i)]->color;
        vertexColors.push_back({color.redF(), color.greenF(), color.blueF()});
    }

    m_shape.init(vertices, faces, vertexColors);

    computeAdjacency();
    this->remap = vector<int>(vertices.size());
    this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
    this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
    this->cached_positions = vertices;
}

void ARAP::simplify(Settings& settings) {
    std::string dirPath = settings.meshPath.substr(0, settings.meshPath.rfind(".")) + "/";
    mesh.simplify(settings.simplifyTarget, dirPath);

    const vector<Vector3f>& vertices = mesh.getVertices();
    const vector<Vector3i>& faces = mesh.getFaces();

    vector<Vector3f> vertexColors; vertexColors.reserve(vertices.size());
    for(int i = 0; i < vertices.size(); i++) {
        auto& color = settings.orientationGroups[this->mesh.getOrientationGroup(i)]->color;
        vertexColors.push_back({color.redF(), color.greenF(), color.blueF()});
    }

    m_shape.init(vertices, faces, vertexColors);

    computeAdjacency();
    this->remap = vector<int>(vertices.size());
    this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
    this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
    this->cubeData = vector<CubeData>(vertices.size(), CubeData{});
    this->cached_positions = vertices;
}

bool ARAP::expand(int toLevel, Settings& settings) {
    if(mesh.expand(toLevel)) {
        this->modified = true;

        const vector<Vector3f>& vertices = mesh.getVertices();
        const vector<Vector3i>& faces = mesh.getFaces();

        vector<Vector3f> vertexColors; vertexColors.reserve(vertices.size());
        for(int i = 0; i < vertices.size(); i++) {
            auto& color = settings.orientationGroups[this->mesh.getOrientationGroup(i)]->color;
            vertexColors.push_back({color.redF(), color.greenF(), color.blueF()});
        }

        m_shape.init(vertices, faces, vertexColors);
        computeAdjacency();
        this->remap = vector<int>(vertices.size());
        this->W = SparseMatrix<float>(this->adj.size(), this->adj.size());
        this->rotations = vector<Matrix3f>(vertices.size(), Matrix3f::Identity());
        this->cubeData = vector<CubeData>(vertices.size(), CubeData{});
        this->cached_positions = vertices;
        this->precompute();
        return true;
    }

    return false;
}

void ARAP::clearActiveGroup(Settings& settings) {
    if(settings.activeGroup != -1) {
        const int res = this->mesh.getVertices().size();
        vector<Vector3f> vertexColors; vertexColors.reserve(res);

        for(int i = 0; i < res; i++) {
            if(this->mesh.getOrientationGroup(i) == settings.activeGroup) {
                this->mesh.setOrientationGroup(i, 0);
            }

            auto& color = settings.orientationGroups[this->mesh.getOrientationGroup(i)]->color;
            vertexColors.push_back({color.redF(), color.greenF(), color.blueF()});
        }

        this->m_shape.setVertexColors(vertexColors);
    }
}

void ARAP::updateVertexColors(Settings& settings) {
    const int res = this->mesh.getVertices().size();
    vector<Vector3f> vertexColors; vertexColors.reserve(res);
    for(int i = 0; i < res; i++) {
        auto& color = settings.orientationGroups[this->mesh.getOrientationGroup(i)]->color;
        vertexColors.push_back({color.redF(), color.greenF(), color.blueF()});
    }

    this->m_shape.setVertexColors(vertexColors);
}

void ARAP::saveMesh(const std::string& filePath) {
    ofstream outfile;
    outfile.open(filePath);

    std::map<int, int> outputVertices;

    const auto& verts = m_shape.getVertices();
    const auto& faces = m_shape.getFaces();

    for(int i = 0; i < verts.size(); i++) {
        const Eigen::Vector3f& v = verts[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    for(int i = 0; i < faces.size(); i++) {
        const Vector3i& f = faces[i];
        int V1 = f[0];
        int V2 = f[1];
        int V3 = f[2];

        outfile << "f " << V1 + 1 << " " << V2 + 1 << " " << V3 + 1 << endl;
    }

    outfile.close();
}
