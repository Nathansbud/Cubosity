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

void Mesh::saveToFile(const string &filePath)
{
    ofstream outfile;
    outfile.open(filePath);

    _vertices.clear();
    _faces.clear();

    HalfEdge::validate(_halfEdges);
    HalfEdge::toVerts(_halfEdges, _vertices, _faces, _indices);

    // Write vertices
    for (size_t i = 0; i < _vertices.size(); i++)
    {
        const Vector3f &v = _vertices[i];
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;
    }

    // Write faces
    for (size_t i = 0; i < _faces.size(); i++)
    {
        const Vector3i &f = _faces[i];
        outfile << "f " << (f[0]+1) << " " << (f[1]+1) << " " << (f[2]+1) << endl;
    }

    outfile.close();
}

void Mesh::saveProgressiveFile(const string &filePath, const HalfEdge::CollapseSequence& cs) {
    ofstream outfile;
    outfile.open(filePath);
    if(outfile.fail()) {
        std::cout << "Failed to open: " << filePath << std::endl;
    } else {
        outfile << "I " << cs.initialFaceResolution << " " << cs.finalFaceResolution << endl;

        int cols = 0;
        for(const HalfEdge::CollapseRecord& cr : cs.collapses) {
            outfile << "C " << cr.collapsedEID << " ";
            outfile << "R " << cr.removedEID << " " << cr.removedOrigin.vid << " " << cr.removedOrigin.point[0] << " " << cr.removedOrigin.point[1] << " " << cr.removedOrigin.point[2] << " ";
            outfile << "S " << cr.shiftedEID << " " << cr.shiftedOrigin.vid << " " << cr.shiftedOrigin.point[0] << " " << cr.shiftedOrigin.point[1] << " " << cr.shiftedOrigin.point[2] << " ";
            outfile << "F " << cr.topFID << " " << cr.bottomFID << " ";
            outfile << "W " << cr.wingVIDs.first << " " << cr.wingVIDs.second << " ";
            outfile << "N ";
            for(const auto n : cr.movedEdges) {
                outfile << n << " ";
            }
            outfile << endl;
            cols++;
        }
        std::cout << "Outputted " << cols << " collapse records..." << std::endl;
    }

    outfile.close();
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

//    saveProgressiveFile("/Users/zackamiton/Code/BrownCS/Gradphics/projects/Cubosity/progressive/testing.stamp", cs);

    int numCollapses = cs.collapses.size();
    this->_collapseState = {
        .sequence = cs,
        .detailLevel = numCollapses
    };

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
