#pragma once

#include "interface/OrientationGroup.h"
#include "graphics/shape.h"
#include "mesh/mesh.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include "Eigen/Sparse"

#include <QColor>

#include <vector>
#include <set>
#include <unordered_map>

class Shader;

using Vindex = int;
using Neighbors = std::map<Vindex, std::pair<Vindex, Vindex>>;

// Mesh/ARAP parameter settings controllable via UI;
// these live in GLwidget to make passing down a little bit less annoying
struct Orientation {
    double lambda;
    QColor color;
    Eigen::Matrix3f rotation;
};

struct Settings {
    int simplifyTarget = 500;

    int collapseLevel = 0;

    float denoiseDistance = 2.f;
    float denoiseSigma1 = 1.f;
    float denoiseSigma2 = 1.f;

    bool animateCubing = true;
    bool animateExpand = false;

    int activeGroup = -1;
    std::unordered_map<int, OrientationGroup*> orientationGroups;

    std::string meshPath;
};

struct CubeData {
    Eigen::Vector3f z = Eigen::Vector3f::Random();
    Eigen::Vector3f u = Eigen::Vector3f::Random();
    float rho = 1e-3f;
};

class ARAP
{
private:
    Shape m_shape;

    std::vector<std::map<Vindex, std::pair<Vindex, Vindex>>> adj;
    std::vector<std::vector<int>> faceAdj;

    std::vector<Eigen::Matrix3f> rotations;

    Eigen::SparseMatrix<float> L;
    Eigen::SparseMatrix<float> W;

    bool modified = true;

    // Store our anchors to see if they've changed
    std::unordered_set<int> cached_anchors;
    std::vector<Eigen::Vector3f> cached_positions;

    // Our L matrix has rows, columns removed, so we need to codify a remapping
    std::vector<int> remap;


    // Sal Khan to the rescue to solve our issues
    Eigen::SimplicialLLT<Eigen::SparseMatrix<float>> sal;

    const int NUM_ITERATIONS = 4;

    std::vector<CubeData> cubeData;

    // get vertex normals and vertex areas
    void getPerVertexInfo();

    Eigen::Vector3f getFaceNormal(const Eigen::Vector3i& face);
    float getFaceArea(const Eigen::Vector3i& face);

    std::vector<Eigen::Vector3f> normals;
    std::vector<float> areas;
public:
    ARAP();

    Mesh mesh;
    void subdivide(Settings&);
    void denoise(Settings&);
    void cubify(int iterations, Settings&);
    void simplify(Settings&);
    bool expand(int toLevel, Settings&);

    void init(Eigen::Vector3f &min, Eigen::Vector3f &max, Settings&);
    void computeAdjacency();
    void move(int vertex, Eigen::Vector3f pos);

    // update sparse matrix L and corresponding decomp
    void precompute();
    void computeWeights(const auto& verts);
    void computeRotations(const auto& newVerts, const int moving, const Eigen::Vector3f& moved);
    void computeCubeRotations(const auto& newVerts, Settings&);
    void computeSystem();

    void updateVertexColors(Settings&);
    void updateOrientationGroup(int vIndex, int groupID, Settings& s)  {
        this->mesh.setOrientationGroup(vIndex, groupID);
        updateVertexColors(s);
    }

    int getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold)
    {
        return m_shape.getClosestVertex(start, ray, threshold);
    }

    void draw(Shader *shader, GLenum mode)
    {
        m_shape.draw(shader, mode);
    }

    void resetAnchors() {
        m_shape.clearAnchors();
    }

    SelectMode select(Shader *shader, int vertex)
    {
        return m_shape.select(shader, vertex);
    }

    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode)
    {
        return m_shape.selectWithSpecifiedMode(shader, vertex, mode);
    }

    bool getAnchorPos(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start)
    {
        return m_shape.getAnchorPos(lastSelected, pos, ray, start);
    }

    bool isTextured() {
        return m_shape.isTextured();
    }
};
