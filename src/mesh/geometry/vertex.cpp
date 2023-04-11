#include "vertex.h"

Vertex::Vertex(Vector3f position, Halfedge* he) {
    this->position = position;
    this->halfedge = he;
}
