#pragma once

#include <GL/glew.h>
#include <vector>
#include <unordered_set>
#include <QImage>

#define EIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT

#include "Eigen/StdVector"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix2f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix3i)

#include "Eigen/Dense"

enum SelectMode
{
    None     = 0,
    Anchor   = 1,
    Unanchor = 2
};

class Shader;

class Shape
{
public:
    Shape();

    void init(
        const std::vector<Eigen::Vector3f> &vertices,
        const std::vector<Eigen::Vector3i> &triangles,
        const std::vector<Eigen::Vector2f> &uv,
        const std::vector<Eigen::Vector3f> &vertexColors,
        const std::string texture);
    void init(
        const std::vector<Eigen::Vector3f> &vertices,
        const std::vector<Eigen::Vector3i> &triangles,
        const std::vector<Eigen::Vector2f> &uv,
        const std::vector<Eigen::Vector3f> &vertexColors);
    void init(
        const std::vector<Eigen::Vector3f> &vertices,
        const std::vector<Eigen::Vector3i> &triangles,
        const std::vector<Eigen::Vector3f> &vertexColors);

    void setVertices(const std::vector<Eigen::Vector3f> &vertices);
    void setVertexColors(const std::vector<Eigen::Vector3f> &vertexColors);

    void setModelMatrix(const Eigen::Affine3f &model);

    void draw(Shader *shader, GLenum mode);
    void clearAnchors();
    SelectMode select(Shader *shader, int vertex);
    bool selectWithSpecifiedMode(Shader *shader, int vertex, SelectMode mode);
    int  getClosestVertex(Eigen::Vector3f start, Eigen::Vector3f ray, float threshold);
    bool getAnchorPos(int lastSelected, Eigen::Vector3f& pos, Eigen::Vector3f ray, Eigen::Vector3f start);

    const std::vector<Eigen::Vector3f>& getVertices();
    const std::vector<Eigen::Vector3i>& getFaces();
    const std::unordered_set<int>& getAnchors();

    bool isTextured() {
        return m_textured;
    }

private:
    bool m_initialized = false;
    bool m_textured = false;

    GLuint m_surfaceVao;
    GLuint m_surfaceVbo;
    GLuint m_surfaceIbo;
    QImage m_image;
    GLuint m_texture;

    unsigned int m_numSurfaceVertices;
    unsigned int m_verticesSize;
    float m_red;
    float m_blue;
    float m_green;
    float m_alpha;

    std::vector<Eigen::Vector3i> m_faces;
    std::vector<Eigen::Vector3f> m_vertices;
    std::vector<Eigen::Vector2f> m_uv;
    std::vector<Eigen::Vector3f> m_vertexColors;
    std::unordered_set<int>      m_anchors;

    Eigen::Matrix4f m_modelMatrix;
    int lastSelected = -1;

    // Helpers

    void selectHelper();
    Eigen::Vector3f getNormal(const Eigen::Vector3i& face);
    void updateMesh(const std::vector<Eigen::Vector3i> &triangles,
                    const std::vector<Eigen::Vector3f> &vertices,
                    const std::vector<Eigen::Vector2f> &uv_cords,
                    const std::vector<Eigen::Vector3f> &vertexColors,
                           std::vector<Eigen::Vector3f>& verts,
                           std::vector<Eigen::Vector3f>& normals,
                           std::vector<Eigen::Vector3f>& cols,
                           std::vector<Eigen::Vector2f>& uv);

    void uninit();
};
