#ifndef EDGE_H
#define EDGE_H

class Halfedge;
class Vertex;

#include <memory>

class Edge {
    public:
        Edge(Halfedge* he);
        Halfedge* halfedge;

        // Cached length for isotropic remeshing so as not to have to compute twice (since expensive)
//        float cached_length = 0;
        bool deleted = false;
};

#endif // EDGE_H
