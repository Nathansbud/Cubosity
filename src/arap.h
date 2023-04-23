#pragma once

#include "graphics/shape.h"
#include "mesh/mesh.h"
#include "Eigen/StdList"
#include "Eigen/StdVector"
#include "Eigen/Sparse"

#include <vector>
#include <set>
#include <map>

class Shader;

using Vindex = int;
using Neighbors = std::map<Vindex, std::pair<Vindex, Vindex>>;

// Mesh/ARAP parameter settings controllable via UI;
// these live in GLwidget to make passing down a little bit less annoying
struct Settings {
    float denoiseDistance = 2.f;
    float denoiseSigma1 = 1.f;
    float denoiseSigma2 = 1.f;
};

struct CubeData {
    Eigen::Vector3f z = Eigen::Vector3f::Random();
    Eigen::Vector3f u = Eigen::Vector3f::Random();
    float rho = 1e-4f;
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
    void subdivide();
    void denoise(Settings&);
    void cubify();

    void init(Eigen::Vector3f &min, Eigen::Vector3f &max);
    void computeAdjacency();
    void move(int vertex, Eigen::Vector3f pos);

    // update sparse matrix L and corresponding decomp
    void precompute();
    void computeWeights(const auto& verts);
    void computeRotations(const auto& newVerts, const int moving, const Eigen::Vector3f& moved);
    void computeCubeRotations(const auto& newVerts);
    void computeSystem();

    // ================== Students, If You Choose To Modify The Code Below, It's On You

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
