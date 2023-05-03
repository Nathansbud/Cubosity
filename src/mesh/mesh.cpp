#include "mesh.h"
#include "geometry/halfedge.h"

#include <iostream>
#include <fstream>

#include <QFileInfo>
#include <QString>

#include "util/tiny_obj_loader.h"

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const vector<Vector3f> &vertices, const vector<Vector3i> &faces) {
    // Copy vertices and faces into internal vector
    _vertices = vertices;
    _faces    = faces;

    HalfEdge::fromVerts(_vertices, _faces, _halfEdges, _geometry);
    cout << "Max IDs: "
         << _geometry.bounds.VID_MAX << "V, "
         << _geometry.bounds.FID_MAX << "F, "
         << _geometry.bounds.EID_MAX << "E" << endl;
}

void Mesh::updatePositions(const vector<Vector3f>& vertices) {
    std::cout << _geometry.vertices.size() << std::endl;

    for (int i = 0; i < vertices.size(); i++) {

        _geometry.vertices[_indices.vertices[i].VID]->point = vertices[i];
    }
}

void Mesh::saveProgressiveMesh(const string &outputDir) {
    ofstream outfile;
    outfile.open(outputDir + "/mesh.obj");

    ofstream stampfile;
    stampfile.open(outputDir + "/mesh.stamp");

    std::map<int, int> outputVertices;

    int VOUT = 0;
    for(auto [VID, vertexPtr] : _geometry.vertices) {
        if(!vertexPtr) continue;
        Vector3f& v = vertexPtr->point;
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;

        // Output the vertex ID that our mesh corresponds with!
        outputVertices.insert({VID, VOUT});
        stampfile << "v " << VOUT++ << " " << VID << endl;
    }

    int FOUT = 0;
    for(auto [FID, facePtr] : _geometry.faces) {
        if(!facePtr) continue;

        HalfEdge::HalfEdge* hedge = facePtr->halfEdge;
        int V1, V2, V3;

        V1 = hedge->vertex->vid;
        V2 = hedge->next->vertex->vid;
        V3 = hedge->next->next->vertex->vid;

        outfile << "f " << (outputVertices[V1] + 1) << " "
                        << (outputVertices[V2] + 1) << " "
                        << (outputVertices[V3] + 1) << endl;

        stampfile << "f " << FOUT++ << " " << FID << endl;
    }

    // In order for our collapse records to be loaded, we need to save our edge relationship IDs
    for(auto [EID, edgePtr] : _geometry.edges) {
        if(!edgePtr) continue;

        stampfile << "e " << EID << " " << edgePtr->halfEdge->vertex->vid << " "
                                        << edgePtr->halfEdge->twin->vertex->vid << endl;
    }

    for(const HalfEdge::CollapseRecord& cr : _collapseState.sequence.collapses) {
        stampfile << "C " << cr.collapsedEID << " "
                  << "R " << cr.removedEID << " " << cr.removedOrigin.vid << " " << cr.removedOrigin.point[0] << " " << cr.removedOrigin.point[1] << " " << cr.removedOrigin.point[2] << " "
                  << "S " << cr.shiftedEID << " " << cr.shiftedOrigin.vid << " " << cr.shiftedOrigin.point[0] << " " << cr.shiftedOrigin.point[1] << " " << cr.shiftedOrigin.point[2] << " "
                  << "F " << cr.topFID << " " << cr.bottomFID << " "
                  << "W " << cr.wingVIDs.first << " " << cr.wingVIDs.second << " "
                  << "N ";

        for(const auto n : cr.movedEdges) { stampfile << n << " "; }

        stampfile << "A ";
        auto& mat = cr.affineMatrix;
        for(int i = 0; i < mat.cols(); i++) {
            stampfile << mat(0, i) << " " << mat(1, i) << " " << mat(2, i) << " ";
        }

        stampfile << "O ";
        for(int n : cr.neighborOrder) { stampfile << n << " "; }
        stampfile << endl;
    }

    outfile.close();
    stampfile.close();
}

void Mesh::subdivide() {
    std::unordered_set<HalfEdge::HalfEdge*> subdividedMesh;
    HalfEdge::subdivide(_halfEdges, subdividedMesh);

    HalfEdge::deleteMesh(_halfEdges);

    _halfEdges = subdividedMesh;
    HalfEdge::toVerts(_halfEdges, _vertices, _faces, _indices);
}

void Mesh::denoise(const float DIST_THRESH, const float SIGMA_C, const float SIGMA_S) {
    std::unordered_set<HalfEdge::HalfEdge*> denoisedMesh;
    HalfEdge::denoise(_halfEdges, denoisedMesh, DIST_THRESH, SIGMA_C, SIGMA_S);

    HalfEdge::deleteMesh(_halfEdges);

    _halfEdges = denoisedMesh;
    HalfEdge::toVerts(_halfEdges, _vertices, _faces, _indices);
}

void Mesh::simplify(const int n) {
    HalfEdge::CollapseSequence cs;
    HalfEdge::simplify(_halfEdges, n, cs, _geometry);

    int numCollapses = cs.collapses.size();
    this->_collapseState = {
        .sequence = cs,
        .detailLevel = numCollapses
    };

    saveProgressiveMesh("/Users/zackamiton/Code/BrownCS/Gradphics/projects/Cubosity/progressive");
    HalfEdge::toVerts(_halfEdges, _vertices, _faces, _indices);
}

bool Mesh::expand() {
    if(_collapseState.detailLevel > 0) {
        HalfEdge::CollapseRecord cr = _collapseState.sequence.collapses[--_collapseState.detailLevel];        
        HalfEdge::ExpandInfo ei;
        HalfEdge::Vertex* toExpand = _geometry.vertices[cr.shiftedOrigin.vid];

        if(!toExpand) {
            std::cout << "idk what went wrong here buck stacko" << std::endl;
            exit(1);
        }

        HalfEdge::expand(toExpand, cr, ei, _halfEdges, _geometry);
        HalfEdge::validate(_halfEdges);

        HalfEdge::toVerts(_halfEdges, _vertices, _faces, _indices);
        return true;
    }

    return false;
}
