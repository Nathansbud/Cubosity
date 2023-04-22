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

    HalfEdge::fromVerts(_vertices, _faces, _halfEdges, _geomID);
    cout << "Max IDs: " << _geomID.VID_MAX << "V, " << _geomID.FID_MAX << "F, " << _geomID.EID_MAX << "E" << endl;
}

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

    HalfEdge::fromVerts(_vertices, _faces, _halfEdges, _geomID);
    HalfEdge::validate(_halfEdges);

    cout << "Loaded " << _faces.size() << " faces and " << _vertices.size() << " vertices" << endl;
    cout << "Max IDs: " << _geomID.VID_MAX << "V, " << _geomID.FID_MAX << "F, " << _geomID.EID_MAX << "E" << endl;
}

void Mesh::saveToFile(const string &filePath)
{
    ofstream outfile;
    outfile.open(filePath);

    _vertices.clear();
    _faces.clear();

    HalfEdge::validate(_halfEdges);
    HalfEdge::toVerts(_halfEdges, _vertices, _faces);

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
    HalfEdge::toVerts(_halfEdges, _vertices, _faces);
}

void Mesh::denoise(const float DIST_THRESH, const float SIGMA_C, const float SIGMA_S) {
    std::unordered_set<HalfEdge::HalfEdge*> denoisedMesh;
    HalfEdge::denoise(_halfEdges, denoisedMesh, DIST_THRESH, SIGMA_C, SIGMA_S);

    HalfEdge::deleteMesh(_halfEdges);

    _halfEdges = denoisedMesh;
    HalfEdge::toVerts(_halfEdges, _vertices, _faces);
}

void Mesh::simplify(const int n) {
    HalfEdge::CollapseSequence cs;
    HalfEdge::simplify(_halfEdges, n, cs);

    saveProgressiveFile("/Users/zackamiton/Code/BrownCS/Gradphics/projects/Cubosity/progressive/testing.stamp", cs);

    HalfEdge::toVerts(_halfEdges, _vertices, _faces);
}
