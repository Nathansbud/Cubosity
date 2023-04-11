#ifndef ZUTILS_H
#define ZUTILS_H

#include <Eigen/Dense>

#include <random>
#include <iostream>

using namespace Eigen;

std::random_device _RD;
std::mt19937 _GEN(_RD());
std::uniform_real_distribution<> _PDF(0, 1);

float RNG() { return _PDF(_GEN); }

Vector3f reflectAboutNormal(const Vector3f& v, const Vector3f& n) {
    return v - (2 * (v.dot(n)) * n);
}

const float FLOAT_EPSILON = 1e-4f;
const double DOUBLE_EPSILON = 1e-8;

inline bool floatEpsEqual(float a, float b) {
    // If the difference between a and b is less than epsilon, they are equal
    return fabs(a - b) < FLOAT_EPSILON;
}

inline bool doubleEpsEqual(double a, double b)
{
    // If the difference between a and b is less than epsilon, they are equal
    return fabs(a - b) < DOUBLE_EPSILON;
}

//void printTwinlessHedges(std::map<std::tuple<int, int>, Halfedge*>& v) {
//    bool foundTwinless = false;
//    for(auto [k, hedge] : v) {
//        if(!hedge->twin) {
//            std::cout << "(" << std::get<0>(k) << ", " << std::get<1>(k) << ") has no twin!" << std::endl;
//            foundTwinless = true;
//        }
//    }

//    if(!foundTwinless) {
//        std::cout << "Did not find any edges in tuple without twins!" << std::endl;
//    }
//}


#endif // ZUTILS_H
