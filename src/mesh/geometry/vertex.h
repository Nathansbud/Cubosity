#ifndef VERTEX_H
#define VERTEX_H

// Forward declaration so mesh exists;
class Halfedge;

#include <Eigen/Dense>
#include <memory>

using namespace Eigen;

class Vertex {
    public:
        Vertex(Vector3f position, Halfedge* he);
        Vector3f position;
        Halfedge* halfedge;

        // For loop subdivision: vertices that get added via splitting => flagged (unflag at end)
        bool flag = false;
        Vector3f cached;
};

#endif // VERTEX_H
