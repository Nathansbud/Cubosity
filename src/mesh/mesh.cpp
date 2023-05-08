#include "mesh.h"
#include "geometry/halfedge.h"

#include <iostream>
#include <fstream>
#include <charconv>
#include <regex>
#include <filesystem>

#include <QFileInfo>
#include <QString>

using namespace Eigen;
using namespace std;

void Mesh::initFromVectors(const vector<Vector3f> &vertices, const vector<Vector3i> &faces) {
    // Copy vertices and faces into internal vector
    _vertices = vertices;
    _faces    = faces;

    HalfEdge::fromVerts(_vertices, _faces, _halfEdges, _geometry);
    update();
}

void Mesh::updatePositions(const vector<Vector3f>& vertices) {
    std::cout << _geometry.vertices.size() << std::endl;

    for (int i = 0; i < vertices.size(); i++) {
        _geometry.vertices[_indices.vindexToVIDs[i]]->point = vertices[i];
    }
}

std::optional<int> toInt(std::string& str) {
    int num;
    if(std::from_chars(str.data(), str.data() + str.size(), num).ec == std::errc{}) {
        return num;
    }

    return std::nullopt;
}

std::optional<float> toFloat(std::string& str) {
    // Frustratingly, from_chars for floating points does not seem to have been implemented by GCC?
    // Maybe it's a C++23 thing, alas
    try {
        return std::stof(str);
    } catch(const std::invalid_argument& ia) {
        return std::nullopt;
    }
}

bool Mesh::loadProgressiveMesh(const string &stampPath,
                               const std::vector<Eigen::Vector3f> &vertices,
                               const std::vector<Eigen::Vector3i> &faces) {
    ifstream stampfile;
    stampfile.open(stampPath);

    std::string line;

    HalfEdge::ProgressiveRemap remap;

    HalfEdge::CollapseSequence cs;

    std::string token;
    std::regex matchUppercase("[A-Z] ");

    try {
        while(getline(stampfile, line)) {
            if(line[0] == 'v' || line[0] == 'f') {
                std::stringstream ss(line);
                std::vector<std::string> parts; parts.reserve(2);
                // Progress past the token
                getline(ss, token, ' ');
                while(getline(ss, token, ' ')) {
                    parts.push_back(token);
                }

                if(parts.size() != 2) throw std::invalid_argument("A");

                std::optional<int> idx = toInt(parts[0]);
                std::optional<int> gid = toInt(parts[1]);

                if(idx == std::nullopt || gid == std::nullopt) throw std::invalid_argument("B");

                if(line[0] == 'v') {
                    remap.vertices.insert({idx.value(), gid.value()});
                } else if(line[0] == 'f') {
                    remap.faces.insert({idx.value(), gid.value()});
                }
            } else if(line[0] == 'e') {
                std::stringstream ss(line);
                std::vector<std::string> parts; parts.reserve(3);
                // Progress past the token
                getline(ss, token, ' ');
                while(getline(ss, token, ' ')) {
                    parts.push_back(token);
                }

                if(parts.size() != 3) throw::invalid_argument("C");

                std::optional<int> eid = toInt(parts[0]);
                std::optional<int> leftVid = toInt(parts[1]);
                std::optional<int> rightVid = toInt(parts[2]);

                if(eid == std::nullopt || leftVid == std::nullopt || rightVid == std::nullopt) throw std::invalid_argument("D");

                const std::pair<int, int> key = (leftVid < rightVid) ?
                            std::pair<int, int>{leftVid.value(), rightVid.value()} :
                            std::pair<int, int>{rightVid.value(), leftVid.value()};

                remap.vertexEdges.insert({key, eid.value()});
            } else if(line[0] == 'C') {
               std::vector<std::string> sections(
                   std::sregex_token_iterator(line.begin() + 2, line.end(), matchUppercase, -1),
                   std::sregex_token_iterator()
               );

               if(sections.size() != 8) throw std::invalid_argument("Collapse Parts");

               std::optional<int> collapsedEID = toInt(sections[0]);
               if(collapsedEID == std::nullopt) throw std::invalid_argument("CollapsedEID");

               std::stringstream rss(sections[1]);

               std::vector<std::string> removedParts; removedParts.reserve(5);
               while(getline(rss, token, ' ')) {
                   removedParts.push_back(token);
               }

               if(removedParts.size() != 5) throw std::invalid_argument("Removed Parts");

               std::optional<int> removedEID = toInt(removedParts[0]);
               std::optional<int> removedVID = toInt(removedParts[1]);
               std::optional<float> removedX = toFloat(removedParts[2]);
               std::optional<float> removedY = toFloat(removedParts[3]);
               std::optional<float> removedZ = toFloat(removedParts[4]);

               if(removedEID == nullopt || removedVID == nullopt || removedX == nullopt || removedY == nullopt || removedZ == nullopt) {
                   throw std::invalid_argument("Removed");
               }

               std::stringstream sss(sections[2]);
               std::vector<std::string> shiftedParts; shiftedParts.reserve(5);
               while(getline(sss, token, ' ')) {
                   shiftedParts.push_back(token);
               }

               if(shiftedParts.size() != 5) throw std::invalid_argument("Shifted Parts");

               std::optional<int> shiftedEID = toInt(shiftedParts[0]);
               std::optional<int> shiftedVID = toInt(shiftedParts[1]);
               std::optional<float> shiftedX = toFloat(shiftedParts[2]);
               std::optional<float> shiftedY = toFloat(shiftedParts[3]);
               std::optional<float> shiftedZ = toFloat(shiftedParts[4]);

               if(shiftedEID == nullopt || shiftedVID == nullopt || shiftedX == nullopt || shiftedY == nullopt || shiftedZ == nullopt) {
                   throw std::invalid_argument("Shifted");
               }

               std::stringstream fss(sections[3]);

               getline(fss, token, ' ');
               std::optional<int> topFID = toInt(token);

               getline(fss, token, ' ');
               std::optional<int> bottomFID = toInt(token);
               if(topFID == nullopt || bottomFID == nullopt) throw std::invalid_argument("FIDs");

               std::stringstream wss(sections[4]);

               getline(wss, token, ' ');
               std::optional<int> wingTop = toInt(token);

               getline(wss, token, ' ');
               std::optional<int> wingBottom = toInt(token);

               if(wingTop == nullopt || wingBottom == nullopt) throw std::invalid_argument("Wings");

               std::stringstream nss(sections[5]);
               std::vector<std::string> movedParts; movedParts.reserve(6);
               while(getline(nss, token, ' ')) {
                   movedParts.push_back(token);
               }

               std::unordered_set<int> movedEdges;
               for(int i = 0; i < movedParts.size(); i++) {
                   std::optional<int> movedEID = toInt(movedParts[i]);
                   if(movedEID == nullopt) throw std::invalid_argument("Moved");

                   movedEdges.insert(movedEID.value());
               }


               std::stringstream ass(sections[6]);
               std::vector<std::string> affineParts; affineParts.reserve(18);
               while(getline(ass, token, ' ')) {
                   affineParts.push_back(token);
               }

               if(affineParts.size() % 3 != 0) throw std::invalid_argument("Affine Size");

               MatrixXf affineMatrix = MatrixXf(3, affineParts.size() / 3);
               for(int i = 0; i < affineParts.size() / 3; i++) {
                   std::optional<float> colX = toFloat(affineParts[3 * i]);
                   std::optional<float> colY = toFloat(affineParts[3 * i + 1]);
                   std::optional<float> colZ = toFloat(affineParts[3 * i + 2]);

                   if(colX == nullopt || colY == nullopt || colZ == nullopt) throw std::invalid_argument("Affine Column");

                   affineMatrix.col(i) = Vector3f{colX.value(), colY.value(), colZ.value()};
               }

               std::stringstream oss(sections[7]);
               std::vector<std::string> orderParts; orderParts.reserve(6);
               while(getline(oss, token, ' ')) {
                   orderParts.push_back(token);
               }

               if(orderParts.size() != affineMatrix.cols()) throw std::invalid_argument("Affine-Order Mismatch");

               std::vector<int> neighborOrder; neighborOrder.reserve(affineParts.size() / 3);
               for(int i = 0; i < orderParts.size(); i++) {
                   std::optional<int> neighVID = toInt(orderParts[i]);
                   if(neighVID == nullopt) throw std::invalid_argument("Neighbor VID");

                   neighborOrder.push_back(neighVID.value());
               }

               HalfEdge::CollapseRecord cr;

               cr.collapsedEID = collapsedEID.value();
               cr.removedEID = removedEID.value();
               cr.shiftedEID = shiftedEID.value();

               cr.removedOrigin = {
                   .halfEdge = nullptr,
                   .point = {removedX.value(), removedY.value(), removedZ.value()},
                   .vid = removedVID.value()
               };

               cr.shiftedOrigin = {
                   .halfEdge = nullptr,
                   .point = {shiftedX.value(), shiftedY.value(), shiftedZ.value()},
                   .vid = shiftedVID.value()
               };

               cr.topFID = topFID.value();
               cr.bottomFID = bottomFID.value();

               cr.wingVIDs = {wingTop.value(), wingBottom.value()};
               cr.movedEdges = movedEdges;

               cr.affineMatrix = affineMatrix;
               cr.neighborOrder = neighborOrder;

               cs.collapses.push_back(cr);
            } else {
                std::cerr << "Found invalid progressive mesh element; could not make sense of line: " << line << std::endl;
                stampfile.close();
                exit(1);
            }
        }
    } catch(const std::invalid_argument& ia) {
        std::cerr << "Encountered error: " << ia.what() << std::endl;
        std::cerr << "Failed to parse file correctly; encountered malformed token: " << token << " @ line " << line << std::endl;
        stampfile.close();
        return false;
    }

    stampfile.close();

    _vertices = vertices;
    _faces = faces;

    HalfEdge::fromProgressive(_vertices, _faces, _halfEdges, _geometry, remap);

    int numCollapses = cs.collapses.size();
    this->_collapseState = {
        .sequence = cs,
        .detailLevel = numCollapses
    };

    std::cout << cs.collapses.size() << " @ " << numCollapses << std::endl;
    loadedProgressive = true;

    update();

    return true;
}

void Mesh::saveMesh(const string &outputPath) {
    ofstream outfile;
    outfile.open(outputPath);

    std::map<int, int> outputVertices;

    int VOUT = 0;
    for(auto [VID, vertexPtr] : _geometry.vertices) {
        if(!vertexPtr) continue;
        Vector3f& v = vertexPtr->point;
        outfile << "v " << v[0] << " " << v[1] << " " << v[2] << endl;

        // Output the vertex ID that our mesh corresponds with!
        outputVertices.insert({VID, VOUT++});
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
    }

    outfile.close();
}

void Mesh::saveProgressiveMesh(const string &outputDir) {
    filesystem::path outDir = filesystem::path(outputDir);
    if(!filesystem::is_directory(outDir)) filesystem::create_directory(outDir);

    ofstream outfile;
    outfile.open(outDir / "mesh.obj");

    ofstream stampfile;
    stampfile.open(outDir / "mesh.stamp");

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

    this->loadedProgressive = true;
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

void Mesh::simplify(const int n, std::string& outputDir) {
    HalfEdge::CollapseSequence cs;
    HalfEdge::simplify(_halfEdges, n, cs, _geometry);

    int numCollapses = cs.collapses.size();
    this->_collapseState = {
        .sequence = cs,
        .detailLevel = numCollapses
    };

    // If we loaded from progressive mesh, don't save to a new progressive mesh
    if(!loadedProgressive) saveProgressiveMesh(outputDir);

    HalfEdge::toVerts(_halfEdges, _vertices, _faces, _indices);
}

bool Mesh::expand(int toLevel = -1) {
    if(_collapseState.detailLevel > 0) {
        do {
            HalfEdge::CollapseRecord cr = _collapseState.sequence.collapses[--_collapseState.detailLevel];
            HalfEdge::ExpandInfo ei;
            HalfEdge::Vertex* toExpand = _geometry.vertices[cr.shiftedOrigin.vid];

            if(!toExpand) {
                std::cout << "idk what went wrong here buck stacko" << std::endl;
                exit(1);
            }

            HalfEdge::expand(toExpand, cr, ei, _halfEdges, _geometry);
        } while(toLevel >= 0 && _collapseState.detailLevel > toLevel);

        update();
        return true;
    }

    return false;
}
