#ifndef HALFEDGE_H
#define HALFEDGE_H

#include <Eigen/Dense>
#include <Eigen/Sparse>

using namespace Eigen;

#include <iostream>
#include <memory>
#include <map>

#include "vertex.h"
#include "face.h"
#include "edge.h"

class Halfedge {
    public:
        static bool validHalfedge(Halfedge* halfedge);
        static void deleteHalfedge(Halfedge* halfedge);

        Halfedge() {};

        bool nonNull() { return this->twin && this->next && this->edge && this->vertex && this->face; }

        Halfedge* twin = nullptr;
        Halfedge* next = nullptr;
        Edge* edge = nullptr;
        Vertex* vertex = nullptr;
        Face* face = nullptr;


};

#endif // HALFEDGE_H
