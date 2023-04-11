#ifndef FACE_H
#define FACE_H

class Halfedge;

#include <memory>

class Face {
    public:
        Face(Halfedge* he);
        Halfedge* halfedge;
};

#endif // FACE_H
